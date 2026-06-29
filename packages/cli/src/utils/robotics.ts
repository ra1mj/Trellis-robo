/**
 * Robotics domain selection for `trellis init --robotics`.
 *
 * A robotics project picks one or more domains; each maps to a spec directory
 * under `.trellis/spec/robotics/domains/<id>/` that is scaffolded on init. The
 * core robotics docs (cpp-style, cpp-performance, ros2-conventions, dynamics,
 * build-tooling) are always written when robotics is enabled, regardless of
 * domain selection.
 */

/** A selectable robotics domain. Mobile/manipulator/legged are morphologies; */
/** rl/vla are orthogonal capability stacks. */
export type RoboticsDomain = "mobile" | "manipulator" | "legged" | "rl" | "vla";

export interface RoboticsDomainInfo {
  id: RoboticsDomain;
  /** Short label shown in the interactive checkbox. */
  label: string;
}

/** Ordered list driving the init checkbox and `--robot-domain` validation. */
export const ROBOTICS_DOMAINS: readonly RoboticsDomainInfo[] = [
  { id: "mobile", label: "Mobile (wheeled / differential-drive)" },
  { id: "manipulator", label: "Manipulator (serial-chain arm)" },
  { id: "legged", label: "Legged (quadruped / humanoid)" },
  { id: "rl", label: "Reinforcement learning (training / sim2real)" },
  { id: "vla", label: "VLA (vision-language-action)" },
] as const;

const DOMAIN_IDS: ReadonlySet<string> = new Set(
  ROBOTICS_DOMAINS.map((d) => d.id),
);

/** Type guard: is `value` a known robotics domain id? */
export function isRoboticsDomain(value: string): value is RoboticsDomain {
  return DOMAIN_IDS.has(value);
}

/**
 * Normalize raw `--robot-domain` input into known domains.
 * Returns the deduped valid domains (init order) and any unknown tokens so the
 * caller can warn without throwing.
 */
export function parseRoboticsDomains(raw: readonly string[]): {
  domains: RoboticsDomain[];
  unknown: string[];
} {
  const seen = new Set<RoboticsDomain>();
  const unknown: string[] = [];
  for (const token of raw) {
    const value = token.trim().toLowerCase();
    if (!value) continue;
    if (isRoboticsDomain(value)) {
      seen.add(value);
    } else {
      unknown.push(token);
    }
  }
  const domains = ROBOTICS_DOMAINS.map((d) => d.id).filter((id) =>
    seen.has(id),
  );
  return { domains, unknown };
}
