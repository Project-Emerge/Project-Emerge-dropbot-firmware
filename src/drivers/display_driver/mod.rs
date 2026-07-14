mod types;

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use embedded_hal_async::i2c::I2c;
use heapless::String;
use ssd1306_embassy_async::{AsyncSsd1306, DisplaySize, Ready};

use core::fmt::Write as _;

use crate::traits::DisplayController;
pub use types::DisplayError;

type Display<I2C> = AsyncSsd1306<I2C, 1024, Ready>;

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
        ip_address: &str,
        rssi_dbm: Option<i32>,
        network_connected: bool,
    ) -> Result<(), Self::Error> {
        let display = self.display_mut()?;
        let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let mut rssi: String<16> = String::new();

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

        display.flush().await.map_err(DisplayError::Bus)
    }
}
