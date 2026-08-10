/// A page of the on-device menu, i.e. one of the screens the display cycles through.
///
/// Exactly one page is shown at a time; a short press of the power button moves to
/// [`MenuPage::next`], which walks [`MenuPage::ALL`] in order and wraps around at the end.
/// Adding a page therefore means adding a variant, listing it in `ALL`, and drawing it in
/// `tasks::display_controller::draw_page`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum MenuPage {
    /// Wi-Fi link, IP address and MQTT broker session.
    #[default]
    Network,
    /// Pack charge, voltages and charger health.
    Battery,
}

impl MenuPage {
    /// Every page, in the order short presses cycle through them.
    pub const ALL: &'static [Self] = &[Self::Network, Self::Battery];

    /// The page following this one, wrapping around after the last.
    #[must_use]
    pub fn next(self) -> Self {
        let index = Self::ALL.iter().position(|page| *page == self).unwrap_or(0);
        Self::ALL[(index + 1) % Self::ALL.len()]
    }
}
