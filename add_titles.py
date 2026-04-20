import asyncio
import pathlib
import re
import shutil
from google import genai
from google.genai import types

# --- Config ---
SUMMARIES_DIR = pathlib.Path(__file__).parent / "summaries"
OUTPUT_DIR = pathlib.Path(__file__).parent / "summaries_titled"
CONCURRENCY = 4

SYSTEM_INSTRUCTION = (
    "You generate concise, descriptive titles for ECE470 (Robotics) lecture note summaries. "
    "The user will send you the Markdown content of a lecture note. "
    "Reply with ONLY the title itself — no quotes, no prefix like 'Title:', no extra text, no punctuation at the end. "
    "Keep it under 10 words. Focus on the main technical topic (e.g., "
    "'Forward Kinematics and DH Parameters', 'Lyapunov Stability of Adaptive Controllers')."
)

# Windows-illegal filename characters
ILLEGAL_CHARS = re.compile(r'[<>:"/\\|?*\n\r\t]')


def sanitize_title(title: str) -> str:
    """Strip illegal filename chars and trim whitespace/dots."""
    title = title.strip().strip('"\'').strip()
    title = ILLEGAL_CHARS.sub("", title)
    title = title.rstrip(". ")
    return title


async def generate_title(client: genai.Client, md_text: str) -> str:
    config = types.GenerateContentConfig(system_instruction=SYSTEM_INSTRUCTION)
    response = await client.aio.models.generate_content(
        model="gemini-3-flash-preview",
        contents=[types.Part.from_text(text=md_text)],
        config=config,
    )
    return sanitize_title(response.text)


async def process_file(sem: asyncio.Semaphore, client: genai.Client, md_file: pathlib.Path):
    async with sem:
        print(f"[Processing] {md_file.name}")
        try:
            md_text = md_file.read_text(encoding="utf-8")
            title = await generate_title(client, md_text)
            if not title:
                raise ValueError("Empty title generated")

            new_name = f"{md_file.stem} - {title}.md"
            dest = OUTPUT_DIR / new_name

            # Skip if a file with the same original stem already exists
            existing = list(OUTPUT_DIR.glob(f"{md_file.stem} - *.md"))
            if existing:
                print(f"  [SKIP] already titled: {existing[0].name}\n")
                return

            shutil.copy2(md_file, dest)
            print(f"  Saved -> {new_name}\n")
        except Exception as e:
            print(f"  ERROR ({md_file.name}): {e}\n")


async def main():
    OUTPUT_DIR.mkdir(exist_ok=True)

    md_files = sorted(SUMMARIES_DIR.glob("*.md"))
    print(f"Found {len(md_files)} markdown files. Concurrency: {CONCURRENCY}\n")

    client = genai.Client()
    sem = asyncio.Semaphore(CONCURRENCY)

    tasks = [process_file(sem, client, f) for f in md_files]
    await asyncio.gather(*tasks)

    print("All done!")


if __name__ == "__main__":
    asyncio.run(main())
