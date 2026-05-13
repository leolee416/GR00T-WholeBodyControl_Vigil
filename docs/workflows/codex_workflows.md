
# Codex Workflows

This repository uses lightweight Codex-facing workflows to keep agent behavior
consistent.

Workflows may live under `.skills/`.

## neat-freak

Use `neat-freak` after substantial repository changes.

Purpose:

- synchronize documentation
- keep `AGENTS.md` consistent with current repo behavior
- update relevant docs after implementation changes
- summarize important repository changes
- avoid stale instructions

Use cases:

- after adding or changing bridge code
- after changing runtime assumptions
- after changing documentation structure
- before ending a long Codex session
- after resolving a design decision that future agents must know

Expected behavior:

- keep `AGENTS.md` short and contract-focused
- move details into `docs/`
- update `docs/INDEX.md` when adding docs
- avoid expanding benchmark semantics
- preserve protected-area and safety boundaries

Suggested prompt:

```text
Use the neat-freak workflow to sync docs, AGENTS.md, and repository memory after the recent changes.
vigil-bridge
```
## vigil-bridge
Use this workflow when designing or modifying the Vigil bridge.

Checklist:

- keep GR00T runtime-facing
- keep Vigil benchmark-facing
- avoid circular imports
- use plain protocol payloads
- label perception estimates as estimated
- do not return benchmark scores from the bridge
- keep real and MuJoCo modes behind the same API 

Suggested prompt:

```text
Use the vigil-bridge workflow to design or modify the Vigil bridge while preserving the GR00T/Vigil ownership boundary.
```

## safety-check
Use vigil-bridge when designing or modifying the Vigil bridge.

Checklist:

- no automatic robot motion
- halt() exposed and called on exceptions
- conservative speed and timeout defaults
- fail closed on missing state/camera/command connection
- no hardware commands unless explicitly requested
- no edits to protected low-level control paths unless explicitly approved

Suggested prompt:
```text
Use the safety-check workflow before making deploy-related changes.
```