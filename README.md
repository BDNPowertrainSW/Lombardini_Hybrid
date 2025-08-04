# Simulink Project Versioning Guidelines

This guide outlines standards, tools, and best practices for version control in Simulink-based projects. The goal is **stability**, **reproducibility**, and **collaborative development** across models, code, and FMU artifacts.

---

## 📁 Repository Structure

Organize your repository for clarity and scalability:

```
/project-root
├── models/    # Simulink models (.slx)
├── scripts/   # MATLAB scripts and functions (.m)
├── test/      # Test harnesses and regression scripts // To be created
├── data/      # Input/output datasets // To be created
├── doc/       # Design specs and documentation // to be created
└── README.md
```

---

## 🔗 Version Control System (Git)

- Use SmartGit client for version control.
- Commit with **descriptive messages**.
- Submit non-trivial changes via **Pull Requests (PRs)** or **Merge Requests (MRs)**.
- Host on platforms like **GitHub**.

---

## 🛠️ Simulink Model Best Practices

- Use `.slx` format (compressed XML).
- Save models with **local settings**:  
    `File > Model Properties > Model Settings > Save > "Always save model with local settings"`
- **Do not commit**:
    - Autosave files (`*.asv`)
    - Backup files (`*~`)
- Modularize large models using **Model Referencing**.
- Ensure all changes are saved before committing.

---

## ⚙️ Git Integration for Simulink

Set up `.gitattributes` for proper diff/merge:

```gitattributes
*.slx diff=matlab merge=matlab
*.mdl diff=matlab merge=matlab
```

Configure Git to use MATLAB's compare tool:

```bash
git config --global diff.matlab.command "matlab -batch \"slcompare('%a','%b')\""
```

Manual model comparison:

```matlab
slcompare('modelA.slx', 'modelB.slx')
```

---

## ✅ Testing and Validation

- **Simulink Test**: Define and run test cases.
- **Simulink Coverage**: Track test coverage.
- **MATLAB Unit Testing Framework**: Script regression tests.
- Run all tests before submitting or merging changes.
- Store test scripts in `/test/`.

**Automation:**  
Integrate tests into CI pipelines (GitHub Actions, GitLab CI/CD) using MATLAB batch mode.

---

## 📋 Commit Checklist

- Model is saved and consistent.
- No autosave or backup files included.
- All tests pass.
- Changes reflected in `CHANGELOG.md`.
- FMUs are up-to-date or regenerated.
- Key changes annotated in the model.

---

## 📦 FMU File Handling

- FMUs are binary and not diffable.
- Preferably commit with an ID:
    ```bash
    git lfs track "*.fmu"
    ```
- Prefer re-generating FMUs from versioned model sources during CI or release.

---

## 🌱 Branching Strategy

- **main**: Stable, tagged releases
- **develop**: Integration of ongoing work
- **feature/&lt;name&gt;**: New features/components  
- **hotfix/&lt;name&gt;**: Critical fixes on main

### 🧾 Releases & Tagging

- Use Git tags for checkpoints:
    ```bash
    git tag -a v1.0 -m "Initial stable model with torque calibration"
    git push origin v1.0
    ```
- Document releases in `CHANGELOG.md`.

---

## 📚 Documentation Standards

- Annotate key blocks/subsystems in Simulink with purpose and rationale.
- Store high-level design docs in `/doc/`.
- Maintain a clear `README.md` for each major directory.
- Track model and FMU version history in `CHANGELOG.md`.

---

## 🔍 Simulink Model Diff & Merge Tools

- Use MATLAB's visual tools:
    ```matlab
    slcompare('fileA.slx', 'fileB.slx') % Model comparisons
    ```
- MATLAB provides three-way merge tools for `.slx` conflicts.

---

## 🧩 MATLAB Project Integration (Optional)

- Manage environment with MATLAB Projects:
    ```matlab
    slproject.createProjectFromModel('YourModel')
    ```

---

## 🤝 Collaboration Notes

- **Avoid pushing to main directly.**
- Use code reviews for model health, readability, and maintainability.
- Write helpful, minimal, and specific commit messages  
    _e.g._, `"Added torque map lookup table to EngineController"`

---
