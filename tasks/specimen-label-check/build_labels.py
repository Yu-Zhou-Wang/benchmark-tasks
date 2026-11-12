"""Build printed label textures with Pillow; no simulator or downloaded models."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
from specification import SPECIMENS

HERE = Path(__file__).resolve().parent


def label(path, lines, sizes, shape=(768, 512)):
    image = Image.new("RGB", shape, "#faf9f4")
    draw = ImageDraw.Draw(image)
    # Pillow's packaged/default scalable font avoids a machine-specific font path.
    total = sum(sizes) + 18*(len(lines)-1)
    y = (shape[1] - total)/2
    for text, size in zip(lines, sizes):
        font = ImageFont.load_default(size=size)
        box = draw.textbbox((0, 0), text, font=font)
        draw.text(((shape[0] - (box[2]-box[0]))/2 - box[0], y-box[1]), text,
                  font=font, fill="#080808", stroke_width=2, stroke_fill="#080808")
        y += size + 18
    draw.rectangle((4, 4, shape[0]-5, shape[1]-5), outline="#222222", width=5)
    image.save(path)


def main():
    out = HERE / "assets"
    out.mkdir(exist_ok=True)
    for sample in SPECIMENS:
        label(out / f"{sample.key}.png", (sample.name, sample.sample_id), (180, 240), shape=(768,1024))
        label(out / f"{sample.key}_request.png", ("REQUEST", sample.request_name, sample.request_id),
              (110,220,280), shape=(768,1126))
    for tray in ("PROCESS", "QUARANTINE"):
        label(out / f"{tray.lower()}.png", (tray,), (75,), shape=(1024, 256))


if __name__ == "__main__":
    main()
