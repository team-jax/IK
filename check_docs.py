"""Docs vs code: every backticked identifier / file path in the docs must exist; relative links and
AGENTS.md parent references must resolve. Exit code 1 if anything is unresolved.

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
        "motion2/AGENTS.md", "motion2/arm/AGENTS.md", "motion2/실행방법.txt",
        "motion1/.claude/agents/fable-max.md"]
DOCS = [d for d in DOCS if os.path.exists(d)]          # motion1/ is a separate repository
SKIP_DIRS = {"venv", ".git", ".omc", ".omo", "__pycache__", "meshes", "artifacts", "AGENTS_files",
             "README_files", "One_Arm_JAX_URDF_description", ".venv"}
# Mentioned on purpose: history, removed scripts, or words that are not identifiers.
ALLOWED = {"MAX_CURL", "One_Arm_JAX_URDF", "audit_contacts", "trace_carry", "teleop.py", "audit_contacts.py",
           "trace_carry.py", "requirements.txt", "reacher.xml", "record_delivery", "render_frame", "smoke_test",
           "route.json", "scene_audit", ".py", ".sh", ".task", "/wrist.json"}
BASES = [".", "motion1", "motion2", "motion2/arm", "motion1/scripts", "motion1/src/humanoid_grasp_rl",
         "URDF_1006", "URDF_1006/JAXV2_Assembly_description"]

code = ""
for dirpath, dirnames, filenames in os.walk("."):
    dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
    for name in filenames:
        if name.endswith((".py", ".sh", ".xml", ".urdf", ".toml")):
            code += open(os.path.join(dirpath, name), encoding="utf-8", errors="ignore").read() + "\n"

problems = []
for doc in DOCS:
    text = open(doc, encoding="utf-8").read()
    if doc == "motion1/STATUS.md":
        text = text.split("\n---\n")[0]          # below the rule is G1-era history
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
for doc in [d for d in ("README.md", "STATUS.md", "motion1/README.md", "motion1/AGENTS.md", "AGENTS.md") if os.path.exists(d)]:
    for link in re.findall(r"\]\(([^)#\s]+)\)", open(doc, encoding="utf-8").read()):
        if not link.startswith("http") and not os.path.exists(os.path.join(os.path.dirname(doc), link)):
            problems.append((doc, "link", link))

for p in sorted(set(problems)):
    print("UNRESOLVED", *p)
print(f"docs: {len(DOCS)} checked, {len(set(problems))} unresolved")
sys.exit(1 if problems else 0)
