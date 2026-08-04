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

use crate::traits::DisplayController;
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

/// Body area of the firmware-update screen, i.e. everything below the title separator.
const UPDATE_BODY_TOP: i32 = 19;
const UPDATE_BODY_BOTTOM: i32 = DISPLAY_HEIGHT - 3;
/// Widest line that still fits between the borders: `20 * 6 = 120` of the 128 pixels.
const UPDATE_MESSAGE_MAX_CHARS: usize = 20;
const UPDATE_MESSAGE_MAX_LINES: usize = 3;
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

/// Greedily wraps `text` on spaces into at most `UPDATE_MESSAGE_MAX_LINES` lines that fit
/// the panel width. A word too long to fit on its own is hard-split rather than dropped;
/// anything past the last line is truncated.
fn wrap_message(text: &str) -> Vec<&str, UPDATE_MESSAGE_MAX_LINES> {
    let mut lines: Vec<&str, UPDATE_MESSAGE_MAX_LINES> = Vec::new();
    let mut rest = text.trim();

    while !rest.is_empty() && !lines.is_full() {
        // Byte offset of the first character that no longer fits, and of the last space
        // before it -- the preferred break point. Walking `char_indices` rather than
        // slicing by byte keeps this panic-free on non-ASCII input.
        let mut overflow_at = rest.len();
        let mut last_space = None;
        for (index, (offset, character)) in rest.char_indices().enumerate() {
            if index == UPDATE_MESSAGE_MAX_CHARS {
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

    async fn draw_status(
        &mut self,
        device_id: &str,
        ip_address: &str,
        rssi_dbm: Option<i32>,
        network_connected: bool,
        firmware_version: &str,
    ) -> Result<(), Self::Error> {
        let display = self.display_mut()?;
        let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let mut rssi: String<16> = String::new();
        let mut version: String<16> = String::new();

        let _ = write!(version, "v{firmware_version}");

        match rssi_dbm {
            Some(value) => {
                let _ = write!(rssi, "{value} dBm");
            }
            None => {
                let _ = rssi.push_str("n/a");
            }
        }

        display.clear();
        let _ = Rectangle::new(Point::zero(), Size::new(128, 64))
            .into_styled(line_style)
            .draw(display);
        let _ = Line::new(Point::new(0, 15), Point::new(127, 15))
            .into_styled(line_style)
            .draw(display);
        let _ = Line::new(Point::new(0, 32), Point::new(127, 32))
            .into_styled(line_style)
            .draw(display);
        let _ = Line::new(Point::new(0, 49), Point::new(127, 49))
            .into_styled(line_style)
            .draw(display);

        let _ =
            Text::with_baseline("IP", Point::new(4, 3), text_style, Baseline::Top).draw(display);
        let _ = Text::with_baseline(ip_address, Point::new(22, 3), text_style, Baseline::Top)
            .draw(display);
        let _ =
            Text::with_baseline("RSSI", Point::new(4, 19), text_style, Baseline::Top).draw(display);
        let _ = Text::with_baseline(rssi.as_str(), Point::new(40, 19), text_style, Baseline::Top)
            .draw(display);
        let _ =
            Text::with_baseline("NET", Point::new(4, 38), text_style, Baseline::Top).draw(display);
        let _ = Text::with_baseline(
            if network_connected {
                "ONLINE"
            } else {
                "WAITING DHCP"
            },
            Point::new(34, 38),
            text_style,
            Baseline::Top,
        )
        .draw(display);
        let _ =
            Text::with_baseline("ID", Point::new(4, 52), text_style, Baseline::Top).draw(display);
        let _ = Text::with_baseline(device_id, Point::new(22, 52), text_style, Baseline::Top)
            .draw(display);
        // The running firmware version shares the identity row with the device id,
        // right-aligned so it stays put whatever the id's length.
        let _ = Text::with_baseline(
            version.as_str(),
            Point::new(right_aligned_x(version.as_str()), 52),
            text_style,
            Baseline::Top,
        )
        .draw(display);

        display.flush().await.map_err(DisplayError::Bus)
    }

    async fn draw_firmware_update(
        &mut self,
        message: &str,
        percent: Option<u8>,
    ) -> Result<(), Self::Error> {
        const TITLE: &str = "FIRMWARE UPDATE";

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
            TITLE,
            Point::new(centered_x(TITLE), 3),
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
                        Point::new(centered_x(line), UPDATE_BODY_TOP),
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
                        UPDATE_BODY_BOTTOM - LINE_HEIGHT,
                    ),
                    text_style,
                    Baseline::Top,
                )
                .draw(display);
            }
            // Indeterminate: nothing to plot, so the message gets the whole body.
            None => {
                let block_height = lines.len() as i32 * LINE_HEIGHT;
                let mut y = UPDATE_BODY_TOP
                    + ((UPDATE_BODY_BOTTOM - UPDATE_BODY_TOP - block_height) / 2).max(0);

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
