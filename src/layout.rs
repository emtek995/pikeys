use keyberon::layout::{layout, Layers};

// Basic testing layout
#[rustfmt::skip]
pub static LAYERS: Layers<10, 4, 1, ()> = layout! {
{
    [Q W F P G J L U Y SColon],
    [A R S T D H N E I O],
    [Z X C V B K M Comma Dot Bslash],
    [No No LGui LShift BSpace Enter Space RGui No No],
}
};
