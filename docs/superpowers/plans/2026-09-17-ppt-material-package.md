# RoboFuzz PPT Complete Material Package Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a detailed, evidence-traceable Chinese PPT source-material package with editable charts and a self-contained ZIP archive.

**Architecture:** Build one isolated deliverable directory with four complementary Markdown documents, an asset-generation script, SVG/PNG chart pairs, and a README. Derive every number from the proposal, checked-in source, current workspace manifests, experiment reports, or the verified 2026-09-17 GitHub Issue snapshot; keep experimental hits, root-cause candidates, public issues, and confirmed vulnerabilities separate.

**Tech Stack:** Markdown, Python 3 with matplotlib, SVG, PNG, shell verification, ZIP archive.

---

### Task 1: Create the package skeleton and provenance rules

**Files:**
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/00_README.md`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/README.md`

- [ ] **Step 1:** Create the package directories with explicit paths.
- [ ] **Step 2:** Write the package purpose, reading order, 2026-09-17 statistical cutoff, A–E evidence scale, version hierarchy, and statement that the package does not claim 34 confirmed vulnerabilities.
- [ ] **Step 3:** State that the original proposal document is the KPI baseline and the January PPT is an early design-intent source.
- [ ] **Step 4:** Verify both README files contain no placeholder markers or unsupported completion claims.

### Task 2: Build the reusable quantitative data tables

**Files:**
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/03_PPT_DATA_TABLES.md`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/data/chart_data.csv`

- [ ] **Step 1:** Add the five-target SpecBlock and modern OracleIR table: MoveIt2 632/27, TB4 598/18, PX4 v1.17 561/26, SLAM 229/38, Nav2 729/42, totals 2749/151.
- [ ] **Step 2:** Add the six formal KPI rows with exact proposal wording and evidence-qualified status.
- [ ] **Step 3:** Add target/profile, mutation strategy, feedback, experiment, Issue, evidence-level, and proposal-version-difference tables.
- [ ] **Step 4:** Add the 34-Issue snapshot: TB3 6/2, PX4 1/4, MoveIt 6/0, TB4/Create3 9/0, SLAM 0/6, total 22/12.
- [ ] **Step 5:** Add chart-ready CSV rows with columns `chart,category,series,value,note`.
- [ ] **Step 6:** Recalculate all totals from component rows and confirm the totals match the source reports.

### Task 3: Write the complete master material library

**Files:**
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/01_PPT_MATERIALS_MASTER.md`

- [ ] **Step 1:** Write the project background, non-crashing control-logic vulnerability definition, three characteristics, three challenges, and three formal routes.
- [ ] **Step 2:** Map the proposed DSL route to `system_doc → preprocessing → SpecBlock → OracleIR → Agent/contextual Oracle → runtime check`, explicitly distinguishing the compiler prototype from the actual target-specific runtime path.
- [ ] **Step 3:** Compare original RoboFuzz capabilities with user increments: semantic feedback, SeedQueue, target profiles, evidence preservation, and Jazzy modernization.
- [ ] **Step 4:** Describe Foxy and Jazzy evolution with repository/image/container boundaries.
- [ ] **Step 5:** Add detailed platform sections for TB3, PX4 v1.12, MoveIt Foxy, TB4, PX4 v1.17, MoveIt2 Jazzy, SLAM, and Nav2, each with environment, input, Oracle, experiment scale, result, root-cause collapse, Issue state, strongest claim, and forbidden overclaim.
- [ ] **Step 6:** Add public-Issue synthesis, A–E evidence funnel, six-KPI judgment, contribution ownership boundaries, gaps, and next priorities.
- [ ] **Step 7:** Include direct local paths or GitHub links for every important quantitative claim.

### Task 4: Build the evidence index and claim/Q&A guide

**Files:**
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/02_PPT_EVIDENCE_INDEX.md`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/04_PPT_CLAIM_AND_QA_GUIDE.md`

- [ ] **Step 1:** Index proposal, source, profile, test, experiment, issue, replay, and chart evidence with columns for path/link, evidence level, proves, and does not prove.
- [ ] **Step 2:** Mark modern generated OracleIR and much of `system_doc` as current-workspace assets when ignored/untracked; distinguish them from Git-tracked legacy PX4 specs.
- [ ] **Step 3:** Add 30-second and 1-minute summaries, recommended slide titles, and a claim ladder: safe, qualified, prohibited.
- [ ] **Step 4:** Add likely defense questions on originality, compiler vs Agent, four input classes, coverage +20%, 34 issues, root-cause deduplication, Go2 absence, sim-real absence, and test failures, with evidence-based answers.

### Task 5: Generate editable visual assets

**Files:**
- Create: `tools/generate_ppt_material_assets.py`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/method_pipeline.svg`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/method_pipeline.png`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/proposal_alignment.svg`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/proposal_alignment.png`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/specblock_oracleir_counts.svg`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/specblock_oracleir_counts.png`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/experiment_root_cause_funnel.svg`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/experiment_root_cause_funnel.png`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/kpi_status.svg`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/kpi_status.png`
- Copy: `docs/assets/issues_by_platform_status.svg` to the package assets directory
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/assets/issues_by_platform_status.png`

- [ ] **Step 1:** Implement deterministic chart generation using a Chinese-capable font and a white-background blue/teal palette.
- [ ] **Step 2:** Render all six charts as both SVG and at least 1600-pixel-wide PNG.
- [ ] **Step 3:** Add a source/interpretation note inside or immediately below each chart.
- [ ] **Step 4:** Open every PNG for visual inspection and correct clipping, unreadable labels, or ambiguous legends.

### Task 6: Verify and package the deliverable

**Files:**
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17/MANIFEST.txt`
- Create: `deliverables/robofuzz_ppt_materials_2026-09-17.zip`

- [ ] **Step 1:** Scan all Markdown for placeholder strings, broken relative asset links, and forbidden claims such as “34 confirmed vulnerabilities.”
- [ ] **Step 2:** Verify key arithmetic: 2749 SpecBlocks, 151 modern OracleIR, 186 total local OracleIR including 35 legacy PX4, 34 issues, 22 open, 12 closed.
- [ ] **Step 3:** Verify every listed package file exists and every PNG can be decoded.
- [ ] **Step 4:** Generate a sorted manifest with file sizes and SHA-256 hashes.
- [ ] **Step 5:** Create the ZIP with the top-level package directory preserved.
- [ ] **Step 6:** List and test the ZIP, then report its absolute path and contents.
