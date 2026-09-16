# Issue Status Chart Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate an exact, accessible horizontal stacked bar chart for the 34 public Issues and embed it in section eight of the project summary.

**Architecture:** Store the visualization as a dependency-free SVG asset under `docs/assets`, with machine-readable `data-open`, `data-closed`, and `data-total` attributes on each platform group. Embed the SVG through a relative Markdown image link immediately before the existing platform-detail table, retaining the table as the authoritative detailed explanation.

**Tech Stack:** SVG 1.1/XML, Markdown, `xmllint`, `rg`, shell assertions.

---

### Task 1: Create and validate the SVG asset

**Files:**
- Create: `docs/assets/issues_by_platform_status.svg`
- Reference: `docs/superpowers/specs/2026-09-16-issue-status-chart-design.md`

- [ ] **Step 1: Run the precondition check**

Run:

```bash
test -f docs/assets/issues_by_platform_status.svg
```

Expected: exit 1 because the SVG does not exist yet.

- [ ] **Step 2: Create the SVG with exact platform data**

Create `docs/assets/issues_by_platform_status.svg` with this structure and values:

```xml
<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="700"
     viewBox="0 0 1280 700" role="img"
     aria-labelledby="chart-title chart-desc">
  <title id="chart-title">公开 Issue 的平台分布与当前状态</title>
  <desc id="chart-desc">截至 2026 年 9 月 16 日，共 34 个公开 Issue，其中开放 22 个、关闭 12 个。TurtleBot3 为 6 个开放、2 个关闭；PX4 为 1 个开放、4 个关闭；MoveIt/MoveIt2 为 6 个开放；TurtleBot4/Create3 为 9 个开放；slam_toolbox 为 6 个关闭。</desc>
  <rect width="1280" height="700" fill="#FFFFFF"/>
  <rect x="24" y="24" width="1232" height="652" rx="18"
        fill="#FFFFFF" stroke="#DCE3EA" stroke-width="2"/>

  <g font-family="Noto Sans CJK SC, Microsoft YaHei, PingFang SC, sans-serif">
    <text x="80" y="70" font-size="30" font-weight="700" fill="#172235">公开 Issue 的平台分布与当前状态</text>
    <text x="80" y="103" font-size="16" fill="#5D6878">统计时点：2026-09-16　｜　共 34 个：开放 22，关闭 12</text>

    <g aria-label="图例">
      <rect x="930" y="53" width="22" height="16" rx="4" fill="#159A8C"/>
      <text x="962" y="67" font-size="15" fill="#354153">开放</text>
      <rect x="1040" y="53" width="22" height="16" rx="4" fill="#7756C5"/>
      <text x="1072" y="67" font-size="15" fill="#354153">关闭</text>
    </g>

    <g stroke="#E5EAF0" stroke-width="1">
      <line x1="330" y1="135" x2="330" y2="575"/>
      <line x1="410" y1="135" x2="410" y2="575"/>
      <line x1="490" y1="135" x2="490" y2="575"/>
      <line x1="570" y1="135" x2="570" y2="575"/>
      <line x1="650" y1="135" x2="650" y2="575"/>
      <line x1="730" y1="135" x2="730" y2="575"/>
      <line x1="810" y1="135" x2="810" y2="575"/>
      <line x1="890" y1="135" x2="890" y2="575"/>
      <line x1="970" y1="135" x2="970" y2="575"/>
      <line x1="1050" y1="135" x2="1050" y2="575"/>
      <line x1="1130" y1="135" x2="1130" y2="575"/>
    </g>

    <g font-size="14" fill="#687486" text-anchor="middle">
      <text x="330" y="128">0</text><text x="410" y="128">1</text>
      <text x="490" y="128">2</text><text x="570" y="128">3</text>
      <text x="650" y="128">4</text><text x="730" y="128">5</text>
      <text x="810" y="128">6</text><text x="890" y="128">7</text>
      <text x="970" y="128">8</text><text x="1050" y="128">9</text>
      <text x="1130" y="128">10</text>
    </g>
    <text x="730" y="603" font-size="15" fill="#687486" text-anchor="middle">Issue 数量</text>

    <g font-size="18" fill="#263345" text-anchor="end">
      <text x="300" y="184">TurtleBot3</text>
      <text x="300" y="274">PX4</text>
      <text x="300" y="364">MoveIt / MoveIt2</text>
      <text x="300" y="454">TurtleBot4 / Create3</text>
      <text x="300" y="544">slam_toolbox</text>
    </g>

    <g id="turtlebot3" data-open="6" data-closed="2" data-total="8">
      <rect x="330" y="153" width="480" height="46" rx="7" fill="#159A8C"/>
      <rect x="810" y="153" width="160" height="46" rx="7" fill="#7756C5"/>
      <text x="570" y="183" font-size="17" font-weight="700" fill="#FFFFFF" text-anchor="middle">6</text>
      <text x="890" y="183" font-size="17" font-weight="700" fill="#FFFFFF" text-anchor="middle">2</text>
      <text x="990" y="183" font-size="16" font-weight="700" fill="#263345">合计 8</text>
    </g>
    <g id="px4" data-open="1" data-closed="4" data-total="5">
      <rect x="330" y="243" width="80" height="46" rx="7" fill="#159A8C"/>
      <rect x="410" y="243" width="320" height="46" rx="7" fill="#7756C5"/>
      <text x="370" y="273" font-size="17" font-weight="700" fill="#FFFFFF" text-anchor="middle">1</text>
      <text x="570" y="273" font-size="17" font-weight="700" fill="#FFFFFF" text-anchor="middle">4</text>
      <text x="750" y="273" font-size="16" font-weight="700" fill="#263345">合计 5</text>
    </g>
    <g id="moveit" data-open="6" data-closed="0" data-total="6">
      <rect x="330" y="333" width="480" height="46" rx="7" fill="#159A8C"/>
      <text x="570" y="363" font-size="17" font-weight="700" fill="#FFFFFF" text-anchor="middle">6</text>
      <text x="830" y="363" font-size="16" font-weight="700" fill="#263345">合计 6</text>
    </g>
    <g id="turtlebot4-create3" data-open="9" data-closed="0" data-total="9">
      <rect x="330" y="423" width="720" height="46" rx="7" fill="#159A8C"/>
      <text x="690" y="453" font-size="17" font-weight="700" fill="#FFFFFF" text-anchor="middle">9</text>
      <text x="1070" y="453" font-size="16" font-weight="700" fill="#263345">合计 9</text>
    </g>
    <g id="slam-toolbox" data-open="0" data-closed="6" data-total="6">
      <rect x="330" y="513" width="480" height="46" rx="7" fill="#7756C5"/>
      <text x="570" y="543" font-size="17" font-weight="700" fill="#FFFFFF" text-anchor="middle">6</text>
      <text x="830" y="543" font-size="16" font-weight="700" fill="#263345">合计 6</text>
    </g>

    <rect x="80" y="625" width="1120" height="34" rx="8" fill="#FFF7E8" stroke="#F1D59A"/>
    <text x="100" y="648" font-size="14" fill="#725321">说明：公开 Issue 数量反映问题报告和外部沟通情况，不等同于独立漏洞数量或上游确认数量。</text>
  </g>
</svg>
```

- [ ] **Step 3: Validate XML and embedded counts**

Run:

```bash
xmllint --noout docs/assets/issues_by_platform_status.svg
rg -n 'data-open="6" data-closed="2" data-total="8"|data-open="1" data-closed="4" data-total="5"|data-open="6" data-closed="0" data-total="6"|data-open="9" data-closed="0" data-total="9"|data-open="0" data-closed="6" data-total="6"' docs/assets/issues_by_platform_status.svg
```

Expected: `xmllint` exits 0 and `rg` prints all five platform groups.

- [ ] **Step 4: Commit the standalone asset**

```bash
git add -f docs/assets/issues_by_platform_status.svg
git commit -m "docs: add public issue status chart"
```

Expected: the commit contains only the new SVG.

### Task 2: Embed the chart in the project summary

**Files:**
- Modify: `docs/PROJECT_SUMMARY_ALIGNED_WITH_PROPOSAL.md:125`
- Reference: `docs/assets/issues_by_platform_status.svg`

- [ ] **Step 1: Verify the report does not already embed the asset**

Run:

```bash
rg -n 'issues_by_platform_status\.svg' docs/PROJECT_SUMMARY_ALIGNED_WITH_PROPOSAL.md
```

Expected: exit 1 with no matches.

- [ ] **Step 2: Insert the image and caption after the section-eight introduction**

Insert exactly this Markdown after the paragraph ending with “不能相互替代。”:

```markdown
![各平台公开 Issue 的开放与关闭状态分布：TurtleBot3 6 个开放、2 个关闭；PX4 1 个开放、4 个关闭；MoveIt/MoveIt2 6 个开放；TurtleBot4/Create3 9 个开放；slam_toolbox 6 个关闭。](assets/issues_by_platform_status.svg)

*图 1　各平台公开 Issue 的数量与当前状态。数据为 2026 年 9 月 16 日快照；Issue 数量反映问题报告和外部沟通情况，不等同于独立漏洞数或上游确认数。*
```

- [ ] **Step 3: Verify the Markdown link and surrounding placement**

Run:

```bash
test -f docs/assets/issues_by_platform_status.svg
rg -n -B 2 -A 4 'issues_by_platform_status\.svg' docs/PROJECT_SUMMARY_ALIGNED_WITH_PROPOSAL.md
```

Expected: the image appears after the section introduction and before the platform table.

- [ ] **Step 4: Commit the report embedding**

```bash
git add -f docs/PROJECT_SUMMARY_ALIGNED_WITH_PROPOSAL.md
git commit -m "docs: embed issue status chart in project summary"
```

Expected: the commit contains only the project summary.

### Task 3: Run final consistency verification

**Files:**
- Verify: `docs/assets/issues_by_platform_status.svg`
- Verify: `docs/PROJECT_SUMMARY_ALIGNED_WITH_PROPOSAL.md`

- [ ] **Step 1: Validate the SVG and report references**

Run:

```bash
xmllint --noout docs/assets/issues_by_platform_status.svg
test "$(rg -o 'data-open="[0-9]+"' docs/assets/issues_by_platform_status.svg | sed -E 's/[^0-9]//g' | awk '{s+=$1} END {print s}')" = "22"
test "$(rg -o 'data-closed="[0-9]+"' docs/assets/issues_by_platform_status.svg | sed -E 's/[^0-9]//g' | awk '{s+=$1} END {print s}')" = "12"
test "$(rg -o 'data-total="[0-9]+"' docs/assets/issues_by_platform_status.svg | sed -E 's/[^0-9]//g' | awk '{s+=$1} END {print s}')" = "34"
test "$(rg -c 'issues_by_platform_status\.svg' docs/PROJECT_SUMMARY_ALIGNED_WITH_PROPOSAL.md)" = "1"
```

Expected: every command exits 0.

- [ ] **Step 2: Check formatting and repository scope**

Run:

```bash
git diff --check HEAD
git status --short
```

Expected: no whitespace errors; unrelated pre-existing Nav2 and analysis changes remain untouched.
