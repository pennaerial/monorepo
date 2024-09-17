# Penn Aerial Robotics Team Monorepo

Welcome to the monorepo for the Penn Aerial Robotics Team! This repository is designed to centralize all code and documentation for our projects. To ensure a smooth workflow and maintain high code quality, we have set specific guidelines for development.

## Development Workflow

### Branching Strategy
All development should be done on individual branches, not directly on the `main` branch. Please adhere to the following naming convention for your branches:
**Example:**
`user/ethayu/navigation_system`
### Creating a New Branch

Follow these steps to create a new branch and start your project:

1. **Switch to the main branch:**
`git checkout main`
2. **Pull the latest changes from the `main` branch:**
`git pull origin main`
3. **Create and switch to your new branch:**
`git checkout -b user/.../...`
### Development Practices

- **Commit Often:** Make small, incremental commits that include clear and concise descriptions.
- **Write Quality Code:** Ensure your code is clean, well-documented, and adheres to the team’s coding standards.
- **Push Regularly:** Push your changes to the remote repository frequently to keep others updated and avoid conflicts.

### Submitting Changes

1. **Push your branch to the remote repository:**
`git push -u origin user/.../...`
2. **Create a Pull Request (PR):**
- Open the GitHub repository in your browser.
- Navigate to 'Pull Requests' and click 'New Pull Request'.
- Select your branch and provide a detailed description of your changes.

### Code Review

- **Request a Review:** Ask at least one other team member to review your PR.
- **Address Feedback:** Make necessary revisions based on the feedback received.
- **Merge Conflicts:** If there are conflicts, resolve them by merging `main` into your branch and fixing the issues.

### Merging to Main

After your PR has been approved, you may merge it into the `main` branch:

1. **Final Pull from Main:**
`git checkout main`
`git pull`
2. **Merge Your Branch:**
`git merge user/.../...`
`git push origin main`
## Best Practices

- **Stay Informed:** Regularly check the repository for updates and announcements.
- **Communicate:** Keep in touch with your team members about your progress and any challenges you face.
- **Document:** Update project documentation and READMEs as necessary to reflect new changes or additions.

Thank you for contributing to the Penn Aerial Robotics Team. Let's make great things happen together!
