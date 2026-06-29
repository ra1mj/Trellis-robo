import { describe, expect, it, afterEach } from "vitest";

import {
  getBundledSkillTemplates,
  setRoboticsSkillsEnabled,
  ROBOTICS_BUNDLED_SKILLS,
} from "../../src/templates/common/index.js";

function skillNames(): string[] {
  return getBundledSkillTemplates().map((s) => s.name);
}

describe("robotics bundled skill gating", () => {
  afterEach(() => {
    // Reset module-level install flag so other tests see the default.
    setRoboticsSkillsEnabled(false);
  });

  it("excludes ros2-* skills by default", () => {
    setRoboticsSkillsEnabled(false);
    const names = skillNames();
    for (const robo of ROBOTICS_BUNDLED_SKILLS) {
      expect(names).not.toContain(robo);
    }
    // Non-robotics bundled skills stay available.
    expect(names).toContain("trellis-channel");
  });

  it("includes ros2-* skills once robotics is enabled", () => {
    setRoboticsSkillsEnabled(true);
    const names = skillNames();
    for (const robo of ROBOTICS_BUNDLED_SKILLS) {
      expect(names).toContain(robo);
    }
    expect(names).toContain("trellis-channel");
  });

  it("ships each robotics skill with valid SKILL.md frontmatter", () => {
    setRoboticsSkillsEnabled(true);
    const skills = getBundledSkillTemplates().filter((s) =>
      ROBOTICS_BUNDLED_SKILLS.has(s.name),
    );
    expect(skills.length).toBe(ROBOTICS_BUNDLED_SKILLS.size);
    for (const skill of skills) {
      const md = skill.files.find((f) => f.relativePath === "SKILL.md");
      if (!md) throw new Error(`${skill.name} missing SKILL.md`);
      const fm = md.content.match(/^---\n([\s\S]*?)\n---/);
      if (!fm) throw new Error(`${skill.name} SKILL.md missing frontmatter`);
      expect(fm[1]).toMatch(/(^|\n)name:\s*\S/);
      expect(fm[1]).toMatch(/(^|\n)description:\s*\S/);
    }
  });
});
