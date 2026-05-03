---
name: wrapup
description: End-of-session checklist — issues, docs, memory, git. Use when user says "wrap up" or "land the plane".
user_invocable: true
---

# Wrap Up

Update project state to reflect what changed in this session.

## 1. GitHub issues

```bash
gh issue list --repo griswaldbrooks/banshee
```

- Close completed issues with a short reason.
- Open new issues for any discovered work.

## 2. Documentation

| Doc | When to update |
|---|---|
| `CLAUDE.md` | Layout, commands, or conventions changed. |
| `README.md` | User-facing setup or links changed. |
| `website/docs/*.md` | Content changes; check for broken links. |
| `.claude/skills/*/SKILL.md` | Skill behavior changed. |
| `.claude/rules/*.md` | Workflow conventions changed. |

Read before editing — only touch what actually changed.

## 3. Memory

Update `~/.claude/projects/-home-griswald-personal-banshee/memory/`:

- New conventions, tools, or constraints worth remembering across sessions.
- Remove anything that's now wrong.
- Keep `MEMORY.md` index concise (one line per pointer).

## 4. Git

```bash
git status
git diff --stat
```

Don't commit or push without confirmation. If on a feature branch, offer a PR.

## Checklist

- [ ] GitHub issues reflect current state
- [ ] Affected docs updated
- [ ] Memory current and concise
- [ ] No surprise uncommitted work
- [ ] User informed of what changed
