/// What the user did with the power button, as reported by `manage_power_button`.
///
/// The button task acts on the press itself -- only it owns the power latch -- and forwards
/// the event so the display can follow along.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ButtonEvent {
    /// A press released before the power-off threshold: move to the next menu page.
    ShortPress,
    /// The button was held past the power-off threshold. The board is about to cut its own
    /// supply, so this is the last event the display will ever receive.
    LongPress,
}
