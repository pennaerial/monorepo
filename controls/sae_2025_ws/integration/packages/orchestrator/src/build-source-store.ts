import { readFile, writeFile, mkdir } from "node:fs/promises";
import { dirname } from "node:path";

export type BuildSourceKind = "none" | "github" | "local_artifact" | "local_codebase";
export type GithubSource = "release" | "actions";

export interface BuildSourceRecord {
  kind: BuildSourceKind;
  githubSource?: GithubSource;
  tag?: string;
  artifactId?: string;
  runId?: string;
  sha?: string;
  commitSubject?: string;
  name?: string;
  date?: string;
  downloadUrl?: string;
  sizeMb?: number;
  branch?: string;
  artifactName?: string;
  workflowName?: string;
  workflowEvent?: string;
  workflowConclusion?: string;
  localArtifactPath?: string;
  localArtifactSizeBytes?: number;
  codebaseRoot?: string;
  fleetFile?: string;
  updatedAt?: string;
}

const EMPTY_RECORD: BuildSourceRecord = { kind: "none" };

function summarize(record: BuildSourceRecord): string {
  switch (record.kind) {
    case "github":
      if (record.githubSource === "release") return record.name || record.tag || "GitHub release";
      return `GitHub Actions ${record.name || record.artifactName || record.artifactId || "artifact"}`;
    case "local_artifact":
      return record.artifactName || "Local artifact";
    case "local_codebase":
      return record.codebaseRoot || "Local codebase";
    default:
      return "No build source selected";
  }
}

/**
 * Persists which build is currently selected -- single slot, JSON file on
 * disk, same shape as the old dashboard's `BuildSourceStore`. This lives on
 * the orchestrator (not per-client), so it's shared across whichever
 * clients talk to this orchestrator, matching the old system's behavior of
 * one shared selection per backend instance.
 */
export class BuildSourceStore {
  private current: BuildSourceRecord = EMPTY_RECORD;

  constructor(private readonly path: string) {}

  async load(): Promise<void> {
    try {
      const raw = await readFile(this.path, "utf-8");
      this.current = JSON.parse(raw) as BuildSourceRecord;
    } catch {
      this.current = EMPTY_RECORD;
    }
  }

  private async save(): Promise<void> {
    await mkdir(dirname(this.path), { recursive: true });
    await writeFile(this.path, JSON.stringify(this.current, null, 2), "utf-8");
  }

  get(): BuildSourceRecord & { summary: string } {
    return { ...this.current, summary: summarize(this.current) };
  }

  async setGithub(record: Omit<BuildSourceRecord, "kind" | "fleetFile" | "updatedAt">): Promise<void> {
    this.current = {
      ...record,
      kind: "github",
      fleetFile: this.current.fleetFile,
      updatedAt: new Date().toISOString(),
    };
    await this.save();
  }

  async setLocalArtifact(artifactName: string, localArtifactPath: string, sizeBytes: number): Promise<void> {
    this.current = {
      kind: "local_artifact",
      artifactName,
      localArtifactPath,
      localArtifactSizeBytes: sizeBytes,
      fleetFile: this.current.fleetFile,
      updatedAt: new Date().toISOString(),
    };
    await this.save();
  }

  async setLocalCodebase(codebaseRoot: string): Promise<void> {
    this.current = {
      kind: "local_codebase",
      codebaseRoot,
      fleetFile: this.current.fleetFile,
      updatedAt: new Date().toISOString(),
    };
    await this.save();
  }

  async setFleetFile(fleetFile: string): Promise<void> {
    this.current = { ...this.current, fleetFile, updatedAt: new Date().toISOString() };
    await this.save();
  }

  async clear(): Promise<void> {
    this.current = { kind: "none", updatedAt: new Date().toISOString() };
    await this.save();
  }
}
