import { describe, expect, it, beforeEach, afterEach } from "vitest";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";

import { createRoboticsSpecTemplates } from "../../src/configurators/workflow.js";

const CORE_DOCS = [
  "index.md",
  "cpp-style.md",
  "cpp-performance.md",
  "ros2-conventions.md",
  "dynamics.md",
  "build-tooling.md",
];

describe("createRoboticsSpecTemplates", () => {
  let cwd: string;

  beforeEach(() => {
    cwd = fs.mkdtempSync(path.join(os.tmpdir(), "trellis-robo-"));
  });

  afterEach(() => {
    fs.rmSync(cwd, { recursive: true, force: true });
  });

  const roboticsDir = () => path.join(cwd, ".trellis", "spec", "robotics");

  it("writes all core docs and no domain dirs when no domains selected", async () => {
    await createRoboticsSpecTemplates(cwd, []);
    for (const doc of CORE_DOCS) {
      expect(fs.existsSync(path.join(roboticsDir(), doc))).toBe(true);
    }
    expect(fs.existsSync(path.join(roboticsDir(), "domains"))).toBe(false);
  });

  it("writes only the selected domain dirs", async () => {
    await createRoboticsSpecTemplates(cwd, ["mobile", "legged"]);
    const domains = path.join(roboticsDir(), "domains");
    expect(fs.existsSync(path.join(domains, "mobile", "index.md"))).toBe(true);
    expect(fs.existsSync(path.join(domains, "legged", "index.md"))).toBe(true);
    expect(fs.existsSync(path.join(domains, "manipulator"))).toBe(false);
    expect(fs.existsSync(path.join(domains, "rl"))).toBe(false);
    expect(fs.existsSync(path.join(domains, "vla"))).toBe(false);
  });

  it("writes concrete content with no placeholders", async () => {
    await createRoboticsSpecTemplates(cwd, ["vla"]);
    const perf = fs.readFileSync(
      path.join(roboticsDir(), "cpp-performance.md"),
      "utf-8",
    );
    expect(perf.length).toBeGreaterThan(500);
    expect(perf).not.toMatch(/\bTo fill\b|\bTBD\b/);
    const vla = fs.readFileSync(
      path.join(roboticsDir(), "domains", "vla", "index.md"),
      "utf-8",
    );
    expect(vla).toMatch(/^# /);
  });
});
