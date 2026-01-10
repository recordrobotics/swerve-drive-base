<h1 align="center">
  Swerve Drivebase

  <p align="center">
      <a href="https://github.com/recordrobotics/swerve-drive-base/actions/workflows/ci.yml"><img alt="CI" src="https://github.com/recordrobotics/swerve-drive-base/actions/workflows/ci.yml/badge.svg?branch=main"></a>
      <a href="https://docs.recordrobotics.org/"><img alt="Read the Docs" src="https://img.shields.io/readthedocs/2024-control?logo=readthedocs&labelColor=%23556bc2"></a>
      <a href="https://github.com/recordrobotics/swerve-drive-base/actions/workflows/ci.yml"><img alt="Unit Tests Status" src="https://img.shields.io/github/check-runs/recordrobotics/swerve-drive-base/main?nameFilter=JUnit%20Test%20Report&logo=gradle&label=tests&labelColor=purple"></a>
      <a href="https://sonarcloud.io/summary/new_code?id=recordrobotics-swerve-drive-base"><img alt="Quality Gate Status" src="https://sonarcloud.io/api/project_badges/measure?project=recordrobotics-swerve-drive-base&metric=alert_status"></a>
      <a href="https://sonarcloud.io/summary/new_code?id=recordrobotics-swerve-drive-base"><img alt="Coverage" src="https://sonarcloud.io/api/project_badges/measure?project=recordrobotics-swerve-drive-base&metric=coverage"></a>
  </p>
</h1>

Based on the [2025 Moray Code](https://github.com/recordrobotics/2025-moray)

---

## Extensions

This repository recommends the following VS Code extensions for the best development experience:

<table>
  <tr>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker">
        <img src="https://streetsidesoftware.gallerycdn.vsassets.io/extensions/streetsidesoftware/code-spell-checker/4.2.3/1753028947698/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Code Spell Checker"/><br/>
        <b>Code Spell Checker</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=SonarSource.sonarlint-vscode">
        <img src="https://sonarsource.gallerycdn.vsassets.io/extensions/sonarsource/sonarlint-vscode/4.29.0/1755515927519/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="SonarLint"/><br/>
        <b>SonarLint</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=shengchen.vscode-checkstyle">
        <img src="https://shengchen.gallerycdn.vsassets.io/extensions/shengchen/vscode-checkstyle/1.4.2/1680054146028/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Checkstyle for Java"/><br/>
        <b>Checkstyle for Java</b>
      </a>
    </td>
  </tr>
</table>

### SonarLint Setup

To use the custom SonarLint rules configured for this repository, you need to copy the SonarLint rules from [`.vscode/settings.json`](.vscode/settings.json) into your **user settings**:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P` on Mac).
2. Type and select **Preferences: Open User Settings (JSON)**.
3. Copy the `"sonarlint.rules"` section from [`.vscode/settings.json`](.vscode/settings.json) into your user settings file.

This ensures SonarLint uses the same code quality rules as the repository.

-------------------------------------------------
[Record Robotics](https://www.recordrobotics.org/)
