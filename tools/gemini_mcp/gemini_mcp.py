# /// script
# dependencies = [
#     "mcp<2",
#     "google-genai",
#     "python-dotenv",
# ]
# ///
"""Project-scoped Gemini MCP Server for Synapse Humanoid."""
import os
from pathlib import Path

from dotenv import load_dotenv
from google import genai
from google.genai import types
from mcp.server.fastmcp import FastMCP

# Absolute path to .env next to this script
SCRIPT_DIR = Path(__file__).resolve().parent
ENV_PATH = SCRIPT_DIR / ".env"
REPO_ROOT = SCRIPT_DIR.parents[1]

# Load environment configuration
load_dotenv(ENV_PATH)

DEFAULT_MODEL = os.getenv("GEMINI_MODEL", "gemini-2.5-pro")

mcp = FastMCP("gemini")


def get_client() -> tuple[genai.Client | None, str | None]:
    """Retrieve initialized Gemini client using API key from .env."""
    # Re-read in case .env was edited while server was running
    load_dotenv(ENV_PATH, override=True)
    api_key = os.getenv("GEMINI_API_KEY", "").strip()
    if not api_key:
        return None, (
            "Error: GEMINI_API_KEY not configured. "
            f"Please set your key in {ENV_PATH} (copied from {SCRIPT_DIR / '.env.example'})."
        )
    return genai.Client(api_key=api_key), None


@mcp.tool()
def ask_gemini(prompt: str, model: str = "") -> str:
    """Query Google Gemini with a prompt and return the response."""
    client, err = get_client()
    if err:
        return err

    target_model = model.strip() if model.strip() else DEFAULT_MODEL
    try:
        response = client.models.generate_content(
            model=target_model,
            contents=prompt,
        )
        return response.text or "(No text returned from Gemini)"
    except Exception as e:
        return f"Gemini API Error: {e}"


@mcp.tool()
def gemini_chat(messages: list[dict], model: str = "") -> str:
    """Conduct a multi-turn conversation with Google Gemini given a list of messages.

    Each message should have 'role' ('user' or 'model'/'assistant') and 'content'.
    """
    client, err = get_client()
    if err:
        return err

    target_model = model.strip() if model.strip() else DEFAULT_MODEL
    formatted_contents: list[types.Content] = []

    for msg in messages:
        raw_role = str(msg.get("role", "user")).lower()
        role = "model" if raw_role in ("assistant", "model") else "user"
        content_val = msg.get("content", msg.get("parts", ""))
        if isinstance(content_val, list):
            text = " ".join(str(p) for p in content_val)
        else:
            text = str(content_val)
        formatted_contents.append(
            types.Content(
                role=role,
                parts=[types.Part.from_text(text=text)],
            )
        )

    try:
        response = client.models.generate_content(
            model=target_model,
            contents=formatted_contents,
        )
        return response.text or "(No text returned from Gemini)"
    except Exception as e:
        return f"Gemini Chat Error: {e}"


@mcp.tool()
def gemini_review_files(files: list[str], prompt: str, model: str = "") -> str:
    """Review, analyze, or compare files located within the repository using Gemini.

    Access is strictly restricted to files inside the Synapse_Humanoid repository.
    """
    client, err = get_client()
    if err:
        return err

    target_model = model.strip() if model.strip() else DEFAULT_MODEL
    file_sections = []

    for file_str in files:
        raw_path = Path(file_str)
        resolved_path = (REPO_ROOT / raw_path).resolve() if not raw_path.is_absolute() else raw_path.resolve()

        # Strict security sandbox: file must be inside REPO_ROOT
        try:
            rel_path = resolved_path.relative_to(REPO_ROOT)
        except ValueError:
            return f"Security Error: Access denied. '{file_str}' is outside repository root ({REPO_ROOT})."

        if not resolved_path.exists():
            return f"File Error: '{rel_path}' does not exist."
        if not resolved_path.is_file():
            return f"File Error: '{rel_path}' is not a regular file."

        try:
            content = resolved_path.read_text(encoding="utf-8", errors="replace")
        except Exception as e:
            return f"Read Error on '{rel_path}': {e}"

        file_sections.append(f"--- File: {rel_path} ---\n{content}\n")

    combined_prompt = f"{prompt}\n\n" + "\n".join(file_sections)

    try:
        response = client.models.generate_content(
            model=target_model,
            contents=combined_prompt,
        )
        return response.text or "(No text returned from Gemini)"
    except Exception as e:
        return f"Gemini File Review Error: {e}"


if __name__ == "__main__":
    mcp.run()
