/**
 * Integration test for the shipped `import_spec.py` scraper.
 *
 * The python script lives under `src/templates/trellis/scripts/`; this test
 * stamps the scripts into a temp dir with local fixtures (no network) and
 * exercises the real CLI: fetch → managed-region write → idempotent re-run.
 */
import { describe, expect, it, beforeEach, afterEach } from "vitest";
import { execFileSync, spawnSync } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";

let hasPython = true;
try {
  execFileSync("python3", ["--version"], { stdio: "ignore" });
} catch {
  hasPython = false;
}

const SCRIPTS_SRC = path.resolve(
  __dirname,
  "../../src/templates/trellis/scripts",
);

function runImport(cwd: string, args: string[] = []) {
  return spawnSync("python3", [".trellis/scripts/import_spec.py", ...args], {
    cwd,
    encoding: "utf-8",
  });
}

const SPEC_WITH_MARKERS = [
  "# C++ Style",
  "",
  "intro that must survive",
  "",
  "<!-- trellis:managed:style-sources START -->",
  "<!-- placeholder -->",
  "<!-- trellis:managed:style-sources END -->",
  "",
  "footer that must survive",
  "",
].join("\n");

describe.skipIf(!hasPython)("import_spec.py", () => {
  let tmp: string;

  beforeEach(() => {
    tmp = fs.mkdtempSync(path.join(os.tmpdir(), "trellis-import-spec-"));
    fs.cpSync(SCRIPTS_SRC, path.join(tmp, ".trellis", "scripts"), {
      recursive: true,
    });
    fs.mkdirSync(path.join(tmp, ".trellis", "spec", "robotics"), {
      recursive: true,
    });
    fs.mkdirSync(path.join(tmp, "fixtures"), { recursive: true });
    fs.writeFileSync(path.join(tmp, "fixtures", "google.txt"), "style v1\n");
    fs.writeFileSync(
      path.join(tmp, ".trellis", "spec", "robotics", "cpp-style.md"),
      SPEC_WITH_MARKERS,
    );
    fs.writeFileSync(
      path.join(tmp, ".trellis", "spec-sources.json"),
      JSON.stringify({
        sources: [
          {
            id: "google",
            name: "Google C++ Style",
            url: "../fixtures/google.txt",
            target: "cpp-style.md",
            region: "style-sources",
            license: "CC-BY-3.0",
          },
        ],
      }),
    );
  });

  afterEach(() => {
    fs.rmSync(tmp, { recursive: true, force: true });
  });

  const specPath = () =>
    path.join(tmp, ".trellis", "spec", "robotics", "cpp-style.md");

  it("writes provenance into the managed region and preserves surrounding text", () => {
    const result = runImport(tmp);
    expect(result.status).toBe(0);
    const content = fs.readFileSync(specPath(), "utf-8");
    expect(content).toContain("intro that must survive");
    expect(content).toContain("footer that must survive");
    expect(content).toMatch(/Google C\+\+ Style/);
    expect(content).toMatch(/fingerprint: `[0-9a-f]{12}`/);
    expect(content).not.toContain("<!-- placeholder -->");
  });

  it("is idempotent on an unchanged upstream", () => {
    runImport(tmp);
    const first = fs.readFileSync(specPath(), "utf-8");
    runImport(tmp);
    expect(fs.readFileSync(specPath(), "utf-8")).toBe(first);
  });

  it("does not modify files in --dry-run", () => {
    const before = fs.readFileSync(specPath(), "utf-8");
    const result = runImport(tmp, ["--dry-run"]);
    expect(result.status).toBe(0);
    expect(fs.readFileSync(specPath(), "utf-8")).toBe(before);
  });

  it("fails with a clear error on an invalid registry", () => {
    fs.writeFileSync(
      path.join(tmp, ".trellis", "spec-sources.json"),
      JSON.stringify({ sources: [{ id: "x" }] }),
    );
    const result = runImport(tmp);
    expect(result.status).toBe(1);
    expect(result.stderr + result.stdout).toMatch(/missing required/);
  });
});
