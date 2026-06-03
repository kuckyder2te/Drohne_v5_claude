Commit and push all current changes to the remote repository.

Steps:
1. Run `git status` to show what will be committed
2. Run `git diff` (staged + unstaged) to understand what changed
3. Run `git log --oneline -5` to see recent commit style
4. Stage all modified tracked files with `git add` (list files explicitly from git status — do NOT use `git add -A` or `git add .`)
5. Draft a concise German or English commit message (match the language of recent commits) that describes the *why*, not just the *what*. Follow the style of the recent commits.
6. Commit using a HEREDOC so formatting is preserved, appending `Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>`
7. Push to the current remote branch with `git push`
8. Confirm success with `git status`

If there are untracked files (??), ask the user whether to include them before staging.
If the push is to `main` or `master`, warn the user and ask for explicit confirmation first.
