import { describe, expect, it } from "vitest";

import {
  ROBOTICS_DOMAINS,
  isRoboticsDomain,
  parseRoboticsDomains,
} from "../../src/utils/robotics.js";

describe("isRoboticsDomain", () => {
  it("accepts known domains", () => {
    for (const d of ROBOTICS_DOMAINS) {
      expect(isRoboticsDomain(d.id)).toBe(true);
    }
  });

  it("rejects unknown values", () => {
    expect(isRoboticsDomain("drone")).toBe(false);
    expect(isRoboticsDomain("")).toBe(false);
  });
});

describe("parseRoboticsDomains", () => {
  it("returns known domains in canonical init order, not input order", () => {
    const { domains, unknown } = parseRoboticsDomains(["legged", "mobile"]);
    expect(domains).toEqual(["mobile", "legged"]);
    expect(unknown).toEqual([]);
  });

  it("dedupes and is case-insensitive", () => {
    const { domains } = parseRoboticsDomains(["RL", "rl", "Mobile"]);
    expect(domains).toEqual(["mobile", "rl"]);
  });

  it("separates unknown tokens without throwing", () => {
    const { domains, unknown } = parseRoboticsDomains(["mobile", "drone", "arm"]);
    expect(domains).toEqual(["mobile"]);
    expect(unknown).toEqual(["drone", "arm"]);
  });

  it("returns empty arrays for empty input", () => {
    const { domains, unknown } = parseRoboticsDomains([]);
    expect(domains).toEqual([]);
    expect(unknown).toEqual([]);
  });
});
