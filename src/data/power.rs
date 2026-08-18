/// Why the power-latch owner is shutting the board down.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ShutdownReason {
    /// The user held the power button past the shutdown threshold.
    ButtonHeld,
    /// The battery monitor found a critically low pack that is not charging.
    LowBattery,
}

/// User-interface events emitted by the task that owns the power button and latch.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PowerEvent {
    /// A short button press should advance the display menu.
    ShortPress,
    /// The display should show the terminal notice for this shutdown reason.
    ShuttingDown(ShutdownReason),
}
