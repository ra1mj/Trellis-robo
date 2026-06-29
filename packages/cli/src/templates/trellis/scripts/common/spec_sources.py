"""Load and validate the spec-source registry used by import_spec.py.

The registry is JSON (stdlib only — no YAML dependency). Each source declares
where to fetch an upstream standard and which managed region of the robotics
spec pack to refresh.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

REQUIRED_FIELDS = ("id", "name", "url", "target", "region")


class SpecSourceError(Exception):
    """Raised when the registry is missing or malformed."""


def load_sources(config_path: Path) -> list[dict[str, str]]:
    """Parse and validate the registry, returning its `sources` list.

    Raises SpecSourceError on a missing file, invalid JSON, a missing required
    field, or a duplicate source id.
    """
    if not config_path.exists():
        raise SpecSourceError(f"source registry not found: {config_path}")

    try:
        data: Any = json.loads(config_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SpecSourceError(f"invalid JSON in {config_path}: {exc}") from exc

    sources = data.get("sources") if isinstance(data, dict) else None
    if not isinstance(sources, list):
        raise SpecSourceError("registry must contain a 'sources' array")

    seen: set[str] = set()
    validated: list[dict[str, str]] = []
    for index, source in enumerate(sources):
        if not isinstance(source, dict):
            raise SpecSourceError(f"source[{index}] is not an object")
        for field in REQUIRED_FIELDS:
            value = source.get(field)
            if not isinstance(value, str) or not value.strip():
                raise SpecSourceError(
                    f"source[{index}] missing required string field '{field}'"
                )
        source_id = source["id"]
        if source_id in seen:
            raise SpecSourceError(f"duplicate source id: {source_id}")
        seen.add(source_id)
        validated.append(source)

    return validated
