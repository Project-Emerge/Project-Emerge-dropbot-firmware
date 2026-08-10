mod types;

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use embedded_hal_async::i2c::I2c;
use heapless::{String, Vec};
use ssd1306_embassy_async::{AsyncSsd1306, DisplaySize, Ready};

use core::fmt::Write as _;

use crate::data::battery::ChargerFault;
use crate::traits::{BatteryPage, DisplayController, NetworkPage};
pub use types::DisplayError;

type Display<I2C> = AsyncSsd1306<I2C, 1024, Ready>;

const DISPLAY_WIDTH: i32 = 128;
const DISPLAY_HEIGHT: i32 = 64;
/// `FONT_6X10` is drawn without character spacing, so a string's pixel width is simply its
/// character count times this.
const CHAR_WIDTH: i32 = FONT_6X10.character_size.width as i32;
/// Glyph height plus one pixel of leading.
const LINE_HEIGHT: i32 = 11;
/// Distance kept between text and the border rectangle.
const MARGIN: i32 = 4;

/// Baseline of a menu page's title row, and the separator drawn underneath it.
const PAGE_TITLE_TOP: i32 = 3;
const PAGE_SEPARATOR_Y: i32 = 15;
/// Baselines of the four body rows every menu page is laid out on.
const PAGE_ROWS: [i32; 4] = [19, 30, 41, 52];
/// Left edge of the value column, leaving room for a four-character label.
const PAGE_VALUE_X: i32 = 40;
/// Shown in place of an address while the link is down or DHCP has not answered.
const NO_ADDRESS: &str = "---.---.---.---";
/// Shown in place of a reading the charger has not provided.
const NO_READING: &str = "--";

/// Body area of a titled full-screen screen, i.e. everything below the title separator.
const SCREEN_BODY_TOP: i32 = 19;
const SCREEN_BODY_BOTTOM: i32 = DISPLAY_HEIGHT - 3;
/// Widest line that still fits between the borders: `20 * 6 = 120` of the 128 pixels.
const SCREEN_MESSAGE_MAX_CHARS: usize = 20;
const SCREEN_MESSAGE_MAX_LINES: usize = 3;
/// Progress bar geometry. The fillable interior is deliberately 100 pixels wide, so one
/// percent is exactly one pixel.
const PROGRESS_BAR_ORIGIN: Point = Point::new(12, 34);
const PROGRESS_BAR_WIDTH: u32 = 104;
const PROGRESS_BAR_HEIGHT: u32 = 14;
const PROGRESS_FILL_ORIGIN: Point =
    Point::new(PROGRESS_BAR_ORIGIN.x + 2, PROGRESS_BAR_ORIGIN.y + 2);
const PROGRESS_FILL_WIDTH: u32 = 100;
const PROGRESS_FILL_HEIGHT: u32 = PROGRESS_BAR_HEIGHT - 4;

fn text_width(text: &str) -> i32 {
    text.chars().count() as i32 * CHAR_WIDTH
}

fn centered_x(text: &str) -> i32 {
    ((DISPLAY_WIDTH - text_width(text)) / 2).max(0)
}

fn right_aligned_x(text: &str) -> i32 {
    (DISPLAY_WIDTH - MARGIN - text_width(text)).max(0)
}

/// Greedily wraps `text` on spaces into at most `SCREEN_MESSAGE_MAX_LINES` lines that fit
/// the panel width. A word too long to fit on its own is hard-split rather than dropped;
/// anything past the last line is truncated.
fn wrap_message(text: &str) -> Vec<&str, SCREEN_MESSAGE_MAX_LINES> {
    let mut lines: Vec<&str, SCREEN_MESSAGE_MAX_LINES> = Vec::new();
    let mut rest = text.trim();

    while !rest.is_empty() && !lines.is_full() {
        // Byte offset of the first character that no longer fits, and of the last space
        // before it -- the preferred break point. Walking `char_indices` rather than
        // slicing by byte keeps this panic-free on non-ASCII input.
        let mut overflow_at = rest.len();
        let mut last_space = None;
        for (index, (offset, character)) in rest.char_indices().enumerate() {
            if index == SCREEN_MESSAGE_MAX_CHARS {
                overflow_at = offset;
                break;
            }
            if character == ' ' {
                last_space = Some(offset);
            }
        }

        if overflow_at == rest.len() {
            let _ = lines.push(rest);
            break;
        }

        let (line, tail) = rest.split_at(last_space.unwrap_or(overflow_at));
        let _ = lines.push(line.trim_end());
        rest = tail.trim_start();
    }

    lines
}

fn text_style() -> MonoTextStyle<'static, BinaryColor> {
    MonoTextStyle::new(&FONT_6X10, BinaryColor::On)
}

/// Clips `text` to the widest whole number of characters that still fits between `x` and the
/// right border, so an over-long value (a long SSID, say) cannot run past the frame.
fn clip_to_row(text: &str, x: i32) -> &str {
    let max_chars = ((DISPLAY_WIDTH - MARGIN - x) / CHAR_WIDTH).max(0) as usize;
    match text.char_indices().nth(max_chars) {
        Some((offset, _)) => &text[..offset],
        None => text,
    }
}

/// Draws `label` in the label column and `value` in the value column of the row at `y`.
fn draw_label_row<I2C: I2c>(display: &mut Display<I2C>, y: i32, label: &str, value: &str) {
    let _ = Text::with_baseline(label, Point::new(MARGIN, y), text_style(), Baseline::Top)
        .draw(display);
    let _ = Text::with_baseline(
        clip_to_row(value, PAGE_VALUE_X),
        Point::new(PAGE_VALUE_X, y),
        text_style(),
        Baseline::Top,
    )
    .draw(display);
}

/// Draws the row at `y` with `left` against the left border and `right` against the right
/// one. The caller is responsible for the two not overlapping.
fn draw_split_row<I2C: I2c>(display: &mut Display<I2C>, y: i32, left: &str, right: &str) {
    let _ =
        Text::with_baseline(left, Point::new(MARGIN, y), text_style(), Baseline::Top).draw(display);
    let _ = Text::with_baseline(
        right,
        Point::new(right_aligned_x(right), y),
        text_style(),
        Baseline::Top,
    )
    .draw(display);
}

/// Draws the frame every menu page shares: the border, a title row carrying a right-aligned
/// annotation, and the separator below it. Leaves the body rows to the caller.
fn draw_page_frame<I2C: I2c>(display: &mut Display<I2C>, title: &str, annotation: &str) {
    let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    let _ = Rectangle::new(
        Point::zero(),
        Size::new(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32),
    )
    .into_styled(line_style)
    .draw(display);
    draw_split_row(display, PAGE_TITLE_TOP, title, annotation);
    let _ = Line::new(
        Point::new(0, PAGE_SEPARATOR_Y),
        Point::new(DISPLAY_WIDTH - 1, PAGE_SEPARATOR_Y),
    )
    .into_styled(line_style)
    .draw(display);
}

/// Formats a millivolt reading as `8.12V`, i.e. two decimals, truncated.
fn format_volts(millivolts: u16) -> String<8> {
    let mut text = String::new();
    let _ = write!(
        text,
        "{}.{:02}V",
        millivolts / 1000,
        (millivolts % 1000) / 10
    );
    text
}

/// Formats a milliamp reading as `0.45A`, i.e. two decimals, truncated.
fn format_amps(milliamps: u16) -> String<8> {
    let mut text = String::new();
    let _ = write!(text, "{}.{:02}A", milliamps / 1000, (milliamps % 1000) / 10);
    text
}

fn version_label(firmware_version: &str) -> String<16> {
    let mut version = String::new();
    let _ = write!(version, "v{firmware_version}");
    version
}

pub struct SD1306Driver<I2C> {
    i2c: Option<I2C>,
    address: u8,
    display: Option<Display<I2C>>,
}

impl<I2C: I2c> SD1306Driver<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c: Some(i2c),
            address,
            display: None,
        }
    }

    fn display_mut(&mut self) -> Result<&mut Display<I2C>, DisplayError<I2C::Error>> {
        self.display.as_mut().ok_or(DisplayError::NotInitialized)
    }

    /// Draws a framed full-screen screen: `title` centered above a separator, `message`
    /// word-wrapped underneath. With `percent` set, the body makes room for a progress bar
    /// and the figure below it, which leaves only the message's first line room to show.
    async fn draw_screen(
        &mut self,
        title: &str,
        message: &str,
        percent: Option<u8>,
    ) -> Result<(), DisplayError<I2C::Error>> {
        let display = self.display_mut()?;
        let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let fill_style = PrimitiveStyle::with_fill(BinaryColor::On);
        let lines = wrap_message(message);

        display.clear();
        let _ = Rectangle::new(Point::zero(), Size::new(128, 64))
            .into_styled(line_style)
            .draw(display);
        let _ = Text::with_baseline(
            title,
            Point::new(centered_x(title), 3),
            text_style,
            Baseline::Top,
        )
        .draw(display);
        let _ = Line::new(Point::new(0, 16), Point::new(127, 16))
            .into_styled(line_style)
            .draw(display);

        match percent {
            // Known transfer size: the message sits at the top of the body and leaves
            // room for the bar and the figure underneath it.
            Some(percent) => {
                let percent = percent.min(100);
                let mut figure: String<8> = String::new();
                let _ = write!(figure, "{percent}%");

                if let Some(line) = lines.first() {
                    let _ = Text::with_baseline(
                        line,
                        Point::new(centered_x(line), SCREEN_BODY_TOP),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(display);
                }

                let _ = Rectangle::new(
                    PROGRESS_BAR_ORIGIN,
                    Size::new(PROGRESS_BAR_WIDTH, PROGRESS_BAR_HEIGHT),
                )
                .into_styled(line_style)
                .draw(display);
                let _ = Rectangle::new(
                    PROGRESS_FILL_ORIGIN,
                    Size::new(
                        PROGRESS_FILL_WIDTH * u32::from(percent) / 100,
                        PROGRESS_FILL_HEIGHT,
                    ),
                )
                .into_styled(fill_style)
                .draw(display);

                let _ = Text::with_baseline(
                    figure.as_str(),
                    Point::new(
                        centered_x(figure.as_str()),
                        SCREEN_BODY_BOTTOM - LINE_HEIGHT,
                    ),
                    text_style,
                    Baseline::Top,
                )
                .draw(display);
            }
            // Nothing to plot, so the message gets the whole body, vertically centered.
            None => {
                let block_height = lines.len() as i32 * LINE_HEIGHT;
                let mut y = SCREEN_BODY_TOP
                    + ((SCREEN_BODY_BOTTOM - SCREEN_BODY_TOP - block_height) / 2).max(0);

                for line in &lines {
                    let _ = Text::with_baseline(
                        line,
                        Point::new(centered_x(line), y),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(display);
                    y += LINE_HEIGHT;
                }
            }
        }

        display.flush().await.map_err(DisplayError::Bus)
    }
}

impl<I2C: I2c> DisplayController for SD1306Driver<I2C> {
    type Error = DisplayError<I2C::Error>;

    async fn init(&mut self) -> Result<(), Self::Error> {
        if self.display.is_some() {
            return Ok(());
        }

        let i2c = self.i2c.take().ok_or(DisplayError::NotInitialized)?;
        let display = AsyncSsd1306::init(i2c, self.address, DisplaySize::W128H64)
            .await
            .map_err(DisplayError::Bus)?;
        self.display = Some(display);

        Ok(())
    }

    async fn clear(&mut self) -> Result<(), Self::Error> {
        let display = self.display_mut()?;
        display.clear();
        display.flush().await.map_err(DisplayError::Bus)
    }

    async fn draw_text(&mut self, x: u32, y: u32, text: &str) -> Result<(), Self::Error> {
        let x = i32::try_from(x).map_err(|_| DisplayError::CoordinatesOutOfRange)?;
        let y = i32::try_from(y).map_err(|_| DisplayError::CoordinatesOutOfRange)?;
        let display = self.display_mut()?;
        let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        match Text::new(text, Point::new(x, y), style).draw(display) {
            Ok(_) => {}
            Err(error) => match error {},
        }

        display.flush().await.map_err(DisplayError::Bus)
    }

    async fn draw_network_page(&mut self, page: &NetworkPage<'_>) -> Result<(), Self::Error> {
        let version = version_label(page.firmware_version);
        let display = self.display_mut()?;

        display.clear();
        draw_page_frame(
            display,
            "NETWORK",
            if page.ip_address.is_some() {
                "ONLINE"
            } else {
                "NO LINK"
            },
        );
        draw_label_row(display, PAGE_ROWS[0], "SSID", page.ssid);
        draw_label_row(
            display,
            PAGE_ROWS[1],
            "IP",
            page.ip_address.unwrap_or(NO_ADDRESS),
        );
        draw_label_row(display, PAGE_ROWS[2], "MQTT", page.broker_status);
        draw_label_row(display, PAGE_ROWS[3], "ID", page.device_id);
        // The running firmware version shares the identity row, right-aligned so it stays
        // put whatever the id's length.
        let _ = Text::with_baseline(
            version.as_str(),
            Point::new(right_aligned_x(version.as_str()), PAGE_ROWS[3]),
            text_style(),
            Baseline::Top,
        )
        .draw(display);

        display.flush().await.map_err(DisplayError::Bus)
    }

    async fn draw_battery_page(&mut self, page: &BatteryPage<'_>) -> Result<(), Self::Error> {
        const NO_CHARGER: &str = "NO CHARGER DATA";

        let version = version_label(page.firmware_version);
        let display = self.display_mut()?;

        display.clear();

        // Nothing has been read back from the charger yet, or it is not answering at all:
        // say so rather than draw a screenful of dashes.
        let Some(status) = page.status else {
            draw_page_frame(display, "BATTERY", NO_READING);
            let _ = Text::with_baseline(
                NO_CHARGER,
                Point::new(centered_x(NO_CHARGER), PAGE_ROWS[1]),
                text_style(),
                Baseline::Top,
            )
            .draw(display);
            draw_split_row(display, PAGE_ROWS[3], "", version.as_str());
            return display.flush().await.map_err(DisplayError::Bus);
        };

        let mut header: String<12> = String::new();
        let _ = write!(header, "BAT {}%", status.state_of_charge());

        let mut pack: String<16> = String::new();
        let _ = write!(pack, "VBAT {}", format_volts(status.vbat_mv));

        let mut cells: String<20> = String::new();
        let _ = write!(
            cells,
            "{}.{:02}/{}.{:02}V",
            status.cell_top_mv / 1000,
            (status.cell_top_mv % 1000) / 10,
            status.cell_bot_mv / 1000,
            (status.cell_bot_mv % 1000) / 10,
        );

        let mut input: String<16> = String::new();
        let mut input_current: String<8> = String::new();
        if status.input_present() {
            let _ = write!(input, "VBUS {}", format_volts(status.vbus_mv));
            input_current = format_amps(status.input_current_ma);
        } else {
            // Nothing is plugged in, so what the VBUS ADC reads is noise, not a
            // measurement worth putting on screen.
            let _ = write!(input, "VBUS {NO_READING}");
        }

        // The bottom-left slot pairs the charger's own temperature with whatever most
        // deserves attention: a latched fault first, then a pack too hot or cold to charge,
        // then an input that is present but unusable, and otherwise a plain OK.
        let health = status
            .fault
            .map(ChargerFault::label)
            .or_else(|| status.thermistor.label())
            .unwrap_or(if status.input_unusable() {
                "POOR PWR"
            } else {
                "OK"
            });
        let mut footer: String<20> = String::new();
        let _ = write!(footer, "{}C {health}", status.die_temp_dc / 10);

        draw_page_frame(display, header.as_str(), status.state.label());
        draw_split_row(
            display,
            PAGE_ROWS[0],
            pack.as_str(),
            format_amps(status.charge_current_ma).as_str(),
        );
        draw_label_row(display, PAGE_ROWS[1], "CELL", cells.as_str());
        draw_split_row(
            display,
            PAGE_ROWS[2],
            input.as_str(),
            input_current.as_str(),
        );
        draw_split_row(display, PAGE_ROWS[3], footer.as_str(), version.as_str());

        display.flush().await.map_err(DisplayError::Bus)
    }

    async fn draw_firmware_update(
        &mut self,
        message: &str,
        percent: Option<u8>,
    ) -> Result<(), Self::Error> {
        self.draw_screen("FIRMWARE UPDATE", message, percent).await
    }

    async fn draw_notice(&mut self, title: &str, message: &str) -> Result<(), Self::Error> {
        self.draw_screen(title, message, None).await
    }
}
