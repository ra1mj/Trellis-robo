#!/usr/bin/env python3
"""Import authoritative C++/robotics coding standards into the robotics spec pack.

Fetches each source declared in `.trellis/spec-sources.json` and refreshes a
managed region (`<!-- trellis:managed:<region> START/END -->`) inside the target
spec file under `.trellis/spec/robotics/`, writing a provenance entry (name, url,
license, content fingerprint) per source. Content outside the markers is never
touched, and a re-run with unchanged upstream produces no diff (idempotent — the
fingerprint, not a timestamp, is what changes when upstream changes).

Usage:
    python3 import_spec.py [--source ID] [--dry-run]
                           [--config PATH] [--spec-root PATH]

Network: anonymous fetch over https/file; set GH_TOKEN to lift GitHub API rate
limits. Unreachable sources are warned-and-skipped, not fatal.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.log import log_error, log_info, log_success, log_warn
from common.spec_sources import SpecSourceError, load_sources

START_MARKER = "<!-- trellis:managed:{region} START -->"
END_MARKER = "<!-- trellis:managed:{region} END -->"


def fetch_bytes(url: str, base_dir: Path) -> bytes:
    """Fetch a source. Supports http(s)://, file://, and bare local paths
    (resolved relative to the registry file's directory)."""
    if "://" not in url:
        return (base_dir / url).read_bytes()
    request = urllib.request.Request(
        url, headers={"User-Agent": "trellis-import-spec"}
    )
    token = os.environ.get("GH_TOKEN")
    if token and "github" in url:
        request.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(request, timeout=30) as response:
        return response.read()


def fingerprint(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()[:12]


def render_region(entries: list[dict[str, str]]) -> str:
    """Deterministic managed-region body (sorted by id, no timestamp)."""
    lines = [
        "> Sourced standards — managed by `import_spec.py`. Edit outside this region.",
        "",
    ]
    for entry in sorted(entries, key=lambda e: e["id"]):
        lines.append(f"- **{entry['name']}** — <{entry['url']}>")
        lines.append(
            f"  - license: {entry.get('license', 'see source')}"
            f" · fingerprint: `{entry['fingerprint']}`"
        )
    return "\n".join(lines)


def replace_region(text: str, region: str, body: str) -> str | None:
    """Replace the content between the region markers. Returns None if the
    markers are absent or malformed."""
    start = START_MARKER.format(region=region)
    end = END_MARKER.format(region=region)
    start_index = text.find(start)
    end_index = text.find(end)
    if start_index == -1 or end_index == -1 or end_index < start_index:
        return None
    head = text[: start_index + len(start)]
    tail = text[end_index:]
    return f"{head}\n{body}\n{tail}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Import spec standards from GitHub.")
    parser.add_argument("--source", help="Only process this source id")
    parser.add_argument(
        "--dry-run", action="store_true", help="Report changes without writing"
    )
    parser.add_argument(
        "--config",
        default=".trellis/spec-sources.json",
        help="Path to the source registry",
    )
    parser.add_argument(
        "--spec-root",
        default=".trellis/spec/robotics",
        help="Root of the robotics spec pack",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    try:
        sources = load_sources(config_path)
    except SpecSourceError as exc:
        log_error(str(exc))
        return 1

    if args.source:
        sources = [s for s in sources if s["id"] == args.source]
        if not sources:
            log_error(f"no source with id '{args.source}'")
            return 1

    base_dir = config_path.resolve().parent
    spec_root = Path(args.spec_root)

    # Fetch all sources, grouping fingerprinted entries by (target, region).
    groups: dict[tuple[str, str], list[dict[str, str]]] = {}
    fetched = skipped = 0
    for source in sources:
        try:
            data = fetch_bytes(source["url"], base_dir)
        except (urllib.error.URLError, OSError) as exc:
            log_warn(f"skip {source['id']}: fetch failed ({exc})")
            skipped += 1
            continue
        entry = dict(source)
        entry["fingerprint"] = fingerprint(data)
        groups.setdefault((source["target"], source["region"]), []).append(entry)
        fetched += 1

    updated = unchanged = 0
    for (target, region), entries in sorted(groups.items()):
        target_path = spec_root / target
        if not target_path.exists():
            log_warn(f"skip region '{region}': target not found ({target_path})")
            skipped += len(entries)
            continue
        original = target_path.read_text(encoding="utf-8")
        new_text = replace_region(original, region, render_region(entries))
        if new_text is None:
            log_warn(
                f"skip region '{region}': markers missing in {target_path}"
            )
            skipped += len(entries)
            continue
        if new_text == original:
            unchanged += 1
            log_info(f"unchanged: {target}#{region}")
            continue
        updated += 1
        if args.dry_run:
            log_info(f"would update: {target}#{region}")
        else:
            target_path.write_text(new_text, encoding="utf-8")
            log_success(f"updated: {target}#{region}")

    print(
        f"\nfetched {fetched}, updated {updated}, "
        f"unchanged {unchanged}, skipped {skipped}"
    )
    # Non-zero only when every requested source was skipped (nothing worked).
    return 0 if fetched > 0 or not sources else 1


if __name__ == "__main__":
    raise SystemExit(main())
