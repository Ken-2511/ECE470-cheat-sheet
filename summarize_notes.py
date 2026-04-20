import asyncio
import pathlib
from google import genai
from google.genai import types

# --- Config ---
NOTES_DIR = pathlib.Path(__file__).parent / "ECE470"
OUTPUT_DIR = pathlib.Path(__file__).parent / "summaries"

SYSTEM_INSTRUCTION = (
    "You are a helpful teaching assistant for ECE470 (Robotics). "
    "The user will send you images of handwritten lecture notes. "
    "Summarize all the content in these notes into well-structured Markdown. "
    "Use headings, bullet points, and LaTeX math ($ ... $ for inline, $$ ... $$ for display) where appropriate. "
    "Preserve all key definitions, equations, theorems, and examples. "
    "IMPORTANT: Carefully think through the logic and math in the notes. "
    "If you find any logical errors, incorrect equations, or wrong derivations, "
    "point them out clearly and provide the corrected version. "
    "If any steps or explanations are missing or incomplete, fill in the gaps "
    "with the correct content so the summary is complete and self-contained."
)


def load_images(folder: pathlib.Path) -> list[types.Part]:
    """Load all PNG images from a folder, sorted by filename."""
    image_files = sorted(folder.glob("*.png"))
    if not image_files:
        raise FileNotFoundError(f"No PNG images found in {folder}")

    parts = []
    for img_path in image_files:
        data = img_path.read_bytes()
        parts.append(types.Part.from_bytes(data=data, mime_type="image/png"))
        print(f"  Loaded {img_path.name} ({len(data) / 1024:.1f} KB)")
    return parts


CONCURRENCY = 4


async def summarize(client: genai.Client, image_parts: list[types.Part]) -> str:
    """Send images to Gemini and get a Markdown summary (single request, no history)."""
    config = types.GenerateContentConfig(
        system_instruction=SYSTEM_INSTRUCTION,
    )

    contents = [types.Part.from_text(text="Please summarize these lecture notes into Markdown.")]
    contents.extend(image_parts)

    response = await client.aio.models.generate_content(
        model="gemini-3.1-pro-preview",
        contents=contents,
        config=config,
    )
    return response.text


async def process_folder(sem: asyncio.Semaphore, client: genai.Client, folder: pathlib.Path):
    """Process a single folder with concurrency control."""
    output_file = OUTPUT_DIR / f"{folder.name}.md"
    if output_file.exists():
        print(f"[SKIP] {folder.name}")
        return

    async with sem:
        print(f"[Processing] {folder.name}")
        try:
            image_parts = load_images(folder)
            print(f"  Sending {len(image_parts)} images to Gemini...")
            summary = await summarize(client, image_parts)
            output_file.write_text(summary, encoding="utf-8")
            print(f"  Saved -> {output_file.name}\n")
        except Exception as e:
            print(f"  ERROR ({folder.name}): {e}\n")


async def main():
    OUTPUT_DIR.mkdir(exist_ok=True)

    folders = sorted(d for d in NOTES_DIR.iterdir() if d.is_dir())
    print(f"Found {len(folders)} note folders. Concurrency: {CONCURRENCY}\n")

    client = genai.Client()
    sem = asyncio.Semaphore(CONCURRENCY)

    tasks = [process_folder(sem, client, folder) for folder in folders]
    await asyncio.gather(*tasks)

    print("All done!")


if __name__ == "__main__":
    asyncio.run(main())
