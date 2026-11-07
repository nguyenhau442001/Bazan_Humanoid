#!/usr/bin/env python3
"""Generate a high-resolution 1280x640 Social Preview banner for Synapse Humanoid."""

from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
OUT_PATH = REPO / "docs" / "images" / "social_preview.png"
BAZAN_PATH = REPO / "docs" / "images" / "social_preview_bazan.png"
MICO_PATH = REPO / "docs" / "images" / "mico.png"

W, H = 1280, 640


def load_font(size, bold=False):
    """Load system font with cross-platform fallback (macOS Helvetica -> Linux DejaVu -> default)."""
    candidates = []
    if bold:
        candidates.extend([
            ("/System/Library/Fonts/HelveticaNeue.ttc", 1),
            ("/System/Library/Fonts/Supplemental/Helvetica.ttc", 1),
            ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 0),
            ("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", 0),
        ])
    else:
        candidates.extend([
            ("/System/Library/Fonts/HelveticaNeue.ttc", 0),
            ("/System/Library/Fonts/Supplemental/Helvetica.ttc", 0),
            ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 0),
            ("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 0),
        ])

    for path_str, idx in candidates:
        p = Path(path_str)
        if p.exists():
            try:
                if str(p).endswith(".ttc"):
                    return ImageFont.truetype(str(p), size, index=idx)
                return ImageFont.truetype(str(p), size)
            except Exception:
                continue

    try:
        return ImageFont.load_default()
    except Exception:
        return None


def main():
    img = Image.new("RGB", (W, H), (11, 15, 25))
    draw = ImageDraw.Draw(img)

    # 1. Background Gradient (deep obsidian navy to dark slate)
    for y in range(H):
        ratio = y / H
        r = int(10 + 6 * (1.0 - ratio) + 8 * ratio)
        g = int(14 + 8 * (1.0 - ratio) + 16 * ratio)
        b = int(24 + 18 * (1.0 - ratio) + 36 * ratio)
        draw.line([(0, y), (W, y)], fill=(r, g, b))

    # 2. Subtle tech grid
    grid = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    g_draw = ImageDraw.Draw(grid)
    for x in range(0, W, 40):
        g_draw.line([(x, 0), (x, H)], fill=(255, 255, 255, 6))
    for y in range(0, H, 40):
        g_draw.line([(0, y), (W, y)], fill=(255, 255, 255, 6))
    img.paste(Image.alpha_composite(img.convert("RGBA"), grid).convert("RGB"))

    # 3. Ambient Glow behind robot (Kon Tum Blue Palette: #1f5fa8 & #5bb0e8)
    glow = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    glow_draw = ImageDraw.Draw(glow)
    center_x, center_y = 1010, 310
    for rad in range(280, 0, -8):
        alpha = int(48 * (1.0 - (rad / 280.0) ** 1.3))
        # blend Kon Tum blue (31, 95, 168) and light blue (91, 176, 232)
        c_r = int(31 * (rad / 280.0) + 91 * (1.0 - rad / 280.0))
        c_g = int(95 * (rad / 280.0) + 176 * (1.0 - rad / 280.0))
        c_b = int(168 * (rad / 280.0) + 232 * (1.0 - rad / 280.0))
        glow_draw.ellipse(
            [center_x - rad, center_y - rad, center_x + rad, center_y + rad],
            fill=(c_r, c_g, c_b, alpha),
        )

    # Platform ellipse under robot
    glow_draw.ellipse([860, 560, 1160, 610], fill=(91, 176, 232, 40))
    img.paste(Image.alpha_composite(img.convert("RGBA"), glow).convert("RGB"))

    # 4. Fonts
    font_bold = load_font(52, bold=True)
    font_sub = load_font(26, bold=True)
    font_tag = load_font(19, bold=False)
    font_badge = load_font(14, bold=True)
    font_metric_val = load_font(22, bold=True)
    font_metric_lbl = load_font(12, bold=False)
    font_eyebrow = load_font(13, bold=True)

    # Re-obtain draw object on the composite image
    draw = ImageDraw.Draw(img)

    # 5. Left Side Content
    # A. Eyebrow badge
    x_start = 80
    y_pos = 65
    draw.ellipse([x_start, y_pos + 4, x_start + 10, y_pos + 14], fill=(16, 185, 129))
    draw.text(
        (x_start + 18, y_pos),
        "OPEN-SOURCE 20-DOF BIPEDAL HUMANOID PLATFORM",
        font=font_eyebrow,
        fill=(148, 163, 184),
    )

    # B. Main Hero Title
    y_pos += 30
    draw.text((x_start, y_pos), "SYNAPSE HUMANOID", font=font_bold, fill=(255, 255, 255))

    # C. Subtitle (Bazan BZN-01 with Kon Tum Blue)
    y_pos += 65
    draw.text(
        (x_start, y_pos),
        "Bazan (BZN-01) · Next-Gen Locomotion & Digital Twin",
        font=font_sub,
        fill=(91, 176, 232),
    )

    # D. Tagline
    y_pos += 38
    draw.text(
        (x_start, y_pos),
        "Cute outside, capable inside. Designed & built in Vietnam 🇻🇳",
        font=font_tag,
        fill=(203, 213, 225),
    )

    # E. Modular Tech Badges (Pills)
    y_pos += 46
    badges = [
        ("ROS 2 Jazzy", (30, 58, 138), (147, 197, 253), (59, 130, 246)),
        ("MuJoCo 3.8+", (124, 45, 18), (253, 186, 116), (249, 115, 22)),
        ("PyTorch MPS", (136, 19, 55), (253, 164, 175), (244, 63, 94)),
        ("FreeRTOS 200 Hz", (6, 78, 59), (110, 231, 183), (16, 185, 129)),
    ]

    bx = x_start
    for label, bg_color, text_color, border_color in badges:
        bbox = font_badge.getbbox(label)
        bw = bbox[2] - bbox[0] + 24
        bh = 32
        draw.rounded_rectangle(
            [bx, y_pos, bx + bw, y_pos + bh],
            radius=8,
            fill=bg_color,
            outline=border_color,
            width=1,
        )
        draw.text((bx + 12, y_pos + 7), label, font=font_badge, fill=text_color)
        bx += bw + 12

    # F. Key Metric Cards (5 cards in a row) - mass synchronized to 677 g
    y_pos += 56
    cards = [
        ("303 mm", "ROBOT HEIGHT"),
        ("677 g", "TOTAL MASS"),
        ("20 DoF", "ACTUATORS"),
        ("50 Hz", "ONNX POLICY"),
        ("200 Hz", "RT CONTROL"),
    ]

    cw = 125
    ch = 64
    for i, (val, lbl) in enumerate(cards):
        cx = x_start + i * (cw + 12)
        draw.rounded_rectangle(
            [cx, y_pos, cx + cw, y_pos + ch],
            radius=8,
            fill=(22, 31, 48),
            outline=(51, 65, 85),
            width=1,
        )
        draw.text((cx + 12, y_pos + 10), val, font=font_metric_val, fill=(255, 255, 255))
        draw.text((cx + 12, y_pos + 38), lbl, font=font_metric_lbl, fill=(148, 163, 184))

    # G. Secondary footnote
    draw.text(
        (x_start, y_pos + 82),
        "Apache 2.0 License · Apple Silicon M4 Accelerated (769k SPS) · Modular ROS 2",
        font=font_eyebrow,
        fill=(100, 116, 139),
    )

    # 6. Paste Robot on Right
    if MICO_PATH.exists():
        mico = Image.open(MICO_PATH).convert("RGBA")
        aspect = mico.width / mico.height
        mico_h = 560
        mico_w = int(mico_h * aspect)
        mico_resized = mico.resize((mico_w, mico_h), Image.Resampling.LANCZOS)
        img.paste(mico_resized, (1010 - mico_w // 2, 40), mico_resized)

    # 7. Subtle Border around entire canvas
    draw.rectangle([0, 0, W - 1, H - 1], outline=(51, 65, 85), width=2)

    # Save to both social_preview.png and social_preview_bazan.png
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    img.save(OUT_PATH, "PNG", optimize=True)
    img.save(BAZAN_PATH, "PNG", optimize=True)
    print(f"Saved social preview images to {OUT_PATH} and {BAZAN_PATH} ({W}x{H})")


if __name__ == "__main__":
    main()
