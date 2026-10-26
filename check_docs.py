"""Docs vs code: every backticked identifier / file path in the docs must exist; repo paths inside
fenced command blocks must exist; relative links and AGENTS.md parent references must resolve.
Exit code 1 if anything is unresolved.

    venv/bin/python check_docs.py       # from anywhere; motion1/ is skipped when it is not there
"""
import glob
import os
import re
import sys

os.chdir(os.path.dirname(os.path.abspath(__file__)))

DOCS = ["README.md", "STATUS.md", "CLAUDE.md", "AGENTS.md", "haness.md", "URDF_1006/AGENTS.md",
        "URDF_1006/JAXV2_Assembly_description/AGENTS.md", "motion1/README.md", "motion1/AGENTS.md", "motion1/HARNESS.md", "motion1/STATUS.md",
        "motion1/scripts/AGENTS.md", "motion1/src/humanoid_grasp_rl/AGENTS.md",
        "motion2/README.md", "motion2/STATUS.md", "motion2/AGENTS.md", "motion2/src/AGENTS.md",
        "motion2/scripts/AGENTS.md", "motion2/arm/AGENTS.md",
        "motion1/.claude/agents/fable-max.md"]
MISSING = [d for d in DOCS if not d.startswith("motion1/") and not os.path.exists(d)]   # a stale list must not pass silently
DOCS = [d for d in DOCS if os.path.exists(d)]          # motion1/ is a separate repository
SKIP_DIRS = {"venv", ".git", ".omc", ".omo", "__pycache__", "meshes", "artifacts", "AGENTS_files",
             "README_files", ".venv"}
# Mentioned on purpose: history, removed scripts, or words that are not identifiers.
ALLOWED = {"MAX_CURL", "One_Arm_JAX_URDF", "audit_contacts", "trace_carry", "teleop.py", "audit_contacts.py",
           "trace_carry.py", "requirements.txt", "reacher.xml", "record_delivery", "render_frame", "smoke_test",
           "route.json", "scene_audit", ".py", ".sh", ".task", "/wrist.json"}
# Names that only motion1/ defines. motion1/ is a separate repository that team-jax/IK does not hold, so in a
# checkout without it these cannot be verified and are not reported (with motion1/ present nothing is skipped).
MOTION1_ONLY = {"mj_jacSite", "CODE_REVIEW.md", "HARNESS.md", "demonstration.py", "pyproject.toml",
                "watch_grasp.py", "tune_grasp.py", "classify_failures.py", "env.py", "CUBE_GRIP",
                "LIFT_REQUIRED", "JAX_ARM_DIR"}
BASES = [".", "motion1", "motion2", "motion2/src", "motion2/scripts", "motion2/models", "motion2/arm",
         "motion1/scripts", "motion1/src/humanoid_grasp_rl", "URDF_1006", "URDF_1006/JAXV2_Assembly_description"]

code = ""
for dirpath, dirnames, filenames in os.walk("."):
    dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
    for name in filenames:
        if name.endswith((".py", ".sh", ".xml", ".urdf", ".toml")):
            code += open(os.path.join(dirpath, name), encoding="utf-8", errors="ignore").read() + "\n"

CMD_PATH = re.compile(r"(?<![\w./-])((?:motion1|motion2|URDF_1006|venv)/[\w./-]+|\./run_tests\.sh)")
problems = [(d, "missing", "document") for d in MISSING]
for doc in DOCS:
    text = open(doc, encoding="utf-8").read()
    if doc == "motion1/STATUS.md":
        text = text.split("\n---\n")[0]          # below the rule is G1-era history
    for link in re.findall(r"\]\(([^)#\s]+)\)", text):
        if not link.startswith("http") and not os.path.exists(os.path.join(os.path.dirname(doc), link)):
            problems.append((doc, "link", link))
    for block in re.findall(r"```[^\n]*\n(.*?)```", text, re.S):           # commands in fenced blocks
        for path in CMD_PATH.findall(block):
            if "artifacts" not in path and not os.path.exists(path.rstrip("/.,")):   # artifacts/ holds local outputs
                problems.append((doc, "command path", path))
    for token in set(re.findall(r"`([^`\n]+)`", text)):
        token = token.strip()
        if token in ALLOWED:
            continue
        if (re.search(r"\.(py|sh|md|txt|xml|urdf|npz|task|json|toml|mp4|png)$", token) and " " not in token
                and not token.startswith(("http", "--")) and "*" not in token and "..." not in token):
            if not any(os.path.exists(os.path.join(b, token.rstrip("/"))) for b in BASES + [os.path.dirname(doc)]):
                problems.append((doc, "path", token))
        elif (re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*(\(\))?", token) and ("_" in token or token.isupper())
              and len(token) > 3 and token.replace("()", "") not in code):
            problems.append((doc, "identifier", token))

for agents in glob.glob("**/AGENTS.md", recursive=True):
    if agents.startswith("venv"):
        continue
    m = re.search(r"Parent: (\S+)", open(agents, encoding="utf-8").read())
    if m and not os.path.exists(os.path.join(os.path.dirname(agents), m.group(1))):
        problems.append((agents, "parent", m.group(1)))

unchecked = 0
if not os.path.isdir("motion1"):
    kept = [p for p in problems if "motion1/" not in p[2] and p[2] not in MOTION1_ONLY]
    unchecked = len(set(problems)) - len(set(kept))
    problems = kept

for p in sorted(set(problems)):
    print("UNRESOLVED", *p)
print(f"docs: {len(DOCS)} checked, {len(set(problems))} unresolved"
      + (f", {unchecked} references to motion1/ not checked (motion1/ is not here)" if unchecked else ""))
sys.exit(1 if problems else 0)
