"""
Shared config for object types and box assignment.
Generalizes handling for any object label (AI-detected or color-detected).
"""
import hashlib

# Box identifiers for sorting
BOX_A = "A"
BOX_B = "B"

# Optional: explicit mapping for known labels (overrides hash-based)
# Add entries to send specific objects to specific boxes.
LABEL_TO_BOX = {
    "tomato": BOX_A,
    "potato": BOX_B,
    "apple": BOX_A,
    "orange": BOX_B,
    "banana": BOX_A,
    "bottle": BOX_B,
    "cup": BOX_A,
    "cell phone": BOX_B,
    "book": BOX_A,
    "clock": BOX_B,
}

# Object types that can be spawned in simulations (COCO-style names + legacy)
OBJECT_TYPES = [
    "tomato", "potato", "apple", "orange", "banana",
    "bottle", "cup", "cell phone", "book", "clock",
    "person", "car", "dog", "cat", "bird",
]

# BGR colors for OpenCV (cycle by hash for unknown labels)
PALETTE = [
    (0, 0, 255),    # red
    (0, 255, 255),  # yellow
    (255, 0, 0),    # blue
    (0, 255, 0),    # green
    (255, 0, 255),  # magenta
    (255, 255, 0),  # cyan
    (0, 165, 255),  # orange
    (128, 0, 128),  # purple
]

# Matplotlib color names for 3D simulation (same order / hash-based)
PALETTE_MPL = [
    "red", "goldenrod", "blue", "green", "magenta", "cyan", "orange", "purple",
]


def get_box_for_label(label: str) -> str:
    """Return box ID (A or B) for a given object label. Uses config or deterministic hash."""
    label_lower = label.lower().strip()
    if label_lower in LABEL_TO_BOX:
        return LABEL_TO_BOX[label_lower]
    # Deterministic hash for any unknown label
    h = int(hashlib.md5(label_lower.encode()).hexdigest(), 16)
    return BOX_A if (h % 2 == 0) else BOX_B


def get_color_for_label(label: str) -> tuple:
    """Return (B, G, R) color tuple for OpenCV for a given object label (consistent across runs)."""
    label_lower = label.lower().strip()
    h = int(hashlib.md5(label_lower.encode()).hexdigest(), 16)
    idx = h % len(PALETTE)
    return PALETTE[idx]


def get_color_name_for_label(label: str) -> str:
    """Return matplotlib color name for 3D visualization (consistent across runs)."""
    label_lower = label.lower().strip()
    h = int(hashlib.md5(label_lower.encode()).hexdigest(), 16)
    idx = h % len(PALETTE_MPL)
    return PALETTE_MPL[idx]


def get_base_angle_for_box(box_id: str) -> float:
    """Return base angle (degrees) to move arm to the given box."""
    if box_id == BOX_A:
        return 90
    return -90
