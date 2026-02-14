#!/usr/bin/env python3
"""
Autonomous Git Workflow Agent

An intelligent agent that automatically handles Git workflows including:
- Analyzing repository state
- Creating meaningful branch names
- Generating conventional commit messages
- Creating pull requests
"""

import os
import sys
import subprocess
import re
import json
from datetime import datetime
from pathlib import Path


class GitWorkflowAgent:
    def __init__(self):
        self.repo_path = os.getcwd()
        self.branch_pattern = re.compile(r'^[a-z0-9][a-z0-9_-]*[a-z0-9]$')
        
    def run_cmd(self, cmd, capture_output=True, text=True):
        """Run a shell command and return the result."""
        try:
            result = subprocess.run(
                cmd,
                shell=True,
                cwd=self.repo_path,
                capture_output=capture_output,
                text=text
            )
            return result
        except Exception as e:
            print(f"Error running command '{cmd}': {str(e)}")
            return None
    
    def check_repo_state(self):
        """Check if we're in a Git repository and gather state information."""
        # Check if in a Git repo
        result = self.run_cmd("git rev-parse --is-inside-work-tree")
        if not result or result.returncode != 0:
            print("Error: Not in a Git repository")
            return False
            
        # Get current branch
        result = self.run_cmd("git rev-parse --abbrev-ref HEAD")
        if result and result.returncode == 0:
            self.current_branch = result.stdout.strip()
        else:
            print("Error: Could not determine current branch")
            return False
            
        # Get uncommitted changes
        result = self.run_cmd("git status --porcelain")
        if result and result.returncode == 0:
            self.uncommitted_changes = result.stdout.strip().split('\n') if result.stdout.strip() else []
        else:
            print("Error: Could not get Git status")
            return False
            
        # Get remote info
        result = self.run_cmd("git remote -v")
        if result and result.returncode == 0:
            self.remotes = result.stdout.strip().split('\n') if result.stdout.strip() else []
        else:
            print("Warning: Could not get remote information")
            self.remotes = []
            
        # Get recent commit history
        result = self.run_cmd("git log --oneline -5")
        if result and result.returncode == 0:
            self.recent_commits = result.stdout.strip().split('\n') if result.stdout.strip() else []
        else:
            print("Warning: Could not get recent commit history")
            self.recent_commits = []
            
        return True
    
    def analyze_changes(self):
        """Analyze the changes to determine the type of work being done."""
        if not self.uncommitted_changes or self.uncommitted_changes == ['']:
            return {
                'type': 'empty',
                'description': 'No changes detected'
            }
        
        # Get list of changed files
        result = self.run_cmd("git diff --name-only")
        if not result or result.returncode != 0:
            print("Warning: Could not get list of changed files")
            return {'type': 'unknown', 'description': 'Could not analyze changes'}
        
        changed_files = result.stdout.strip().split('\n') if result.stdout.strip() else []
        
        # Categorize changes
        stats_result = self.run_cmd("git diff --stat")
        if stats_result and stats_result.returncode == 0:
            diff_stat = stats_result.stdout
        else:
            diff_stat = ""
        
        # Determine change type based on files modified
        change_info = {
            'type': 'chore',  # default
            'scope': 'unknown',
            'changed_files': changed_files,
            'diff_stat': diff_stat
        }
        
        # Look for specific patterns in changed files
        for file_path in changed_files:
            if 'test' in file_path.lower() or 'spec' in file_path.lower():
                change_info['type'] = 'test'
                break
            elif 'doc' in file_path.lower() or file_path.endswith(('.md', '.txt')):
                change_info['type'] = 'docs'
            elif any(api_dir in file_path for api_dir in ['api', 'routes', 'controllers']):
                change_info['type'] = 'feat'
                change_info['scope'] = 'api'
            elif any(ui_dir in file_path for ui_dir in ['frontend', 'ui', 'components']):
                change_info['type'] = 'feat'
                change_info['scope'] = 'ui'
            elif any(config_dir in file_path for config_dir in ['config', 'conf']):
                change_info['type'] = 'chore'
                change_info['scope'] = 'config'
            elif any(model_dir in file_path for model_dir in ['models', 'schemas']):
                change_info['type'] = 'feat'
                change_info['scope'] = 'models'
            elif any(db_dir in file_path for db_dir in ['db', 'migrations', 'sql']):
                change_info['type'] = 'feat'
                change_info['scope'] = 'database'
            elif any(util_dir in file_path for util_dir in ['utils', 'helpers', 'lib']):
                change_info['type'] = 'refactor'
                change_info['scope'] = 'utils'
            elif any(sec_dir in file_path for sec_dir in ['auth', 'security', 'permissions']):
                change_info['type'] = 'fix'
                change_info['scope'] = 'security'
        
        return change_info
    
    def generate_branch_name(self, change_info):
        """Generate a meaningful branch name based on the changes."""
        if change_info['type'] == 'empty':
            return None
        
        # Determine branch prefix based on change type
        prefixes = {
            'feat': 'feature',
            'fix': 'fix',
            'docs': 'docs',
            'style': 'style',
            'refactor': 'refactor',
            'perf': 'perf',
            'test': 'test',
            'chore': 'chore'
        }
        
        prefix = prefixes.get(change_info['type'], 'feature')
        
        # Generate descriptive name based on files changed
        desc_parts = []
        
        # Look for keywords in file paths to generate description
        for file_path in change_info['changed_files'][:3]:  # Limit to first 3 files
            path_parts = file_path.replace('/', ' ').replace('_', ' ').replace('-', ' ').split()
            for part in path_parts:
                if len(part) > 2 and part not in ['src', 'lib', 'test', 'spec', 'dist', 'build']:
                    # Convert camelCase to space-separated
                    part = re.sub('([a-z0-9])([A-Z])', r'\1 \2', part)
                    desc_parts.append(part.lower())
                    break  # Just take the first meaningful part from each file
        
        if not desc_parts:
            desc_parts = ['update']
        
        # Join and clean the description
        description = '-'.join(desc_parts)[:30]  # Limit length
        description = re.sub(r'[^a-zA-Z0-9-]', '-', description)  # Sanitize
        description = description.strip('-')
        
        branch_name = f"{prefix}/{description}"
        
        # Ensure branch name is valid
        if not self.branch_pattern.match(branch_name):
            # Fallback to a generic name
            timestamp = datetime.now().strftime("%m%d-%H%M")
            branch_name = f"{prefix}/update-{timestamp}"
        
        return branch_name
    
    def generate_commit_message(self, change_info):
        """Generate a conventional commit message based on the changes."""
        if change_info['type'] == 'empty':
            return None
        
        # Map change types to conventional commit types
        type_map = {
            'feat': 'Feature',
            'fix': 'Fix',
            'docs': 'Docs',
            'style': 'Style',
            'refactor': 'Refactor',
            'perf': 'Perf',
            'test': 'Test',
            'chore': 'Chore'
        }
        
        change_type = change_info['type']
        type_display = type_map.get(change_type, 'Update')
        
        # Generate subject based on files changed
        subject_parts = []
        for file_path in change_info['changed_files'][:2]:  # Use first 2 files
            # Extract meaningful part from file path
            path_part = Path(file_path).stem
            if path_part and len(path_part) > 2:
                # Convert camelCase to space-separated
                path_part = re.sub('([a-z0-9])([A-Z])', r'\1 \2', path_part)
                subject_parts.append(path_part.lower())
                break  # Just use the first meaningful file name
        
        if not subject_parts:
            subject_parts = ['changes']
        
        subject = f"{change_type}({subject_parts[0]}): add meaningful changes"
        
        # Generate body explaining the changes
        body_parts = [
            f"This commit introduces changes to improve the {subject_parts[0]} functionality.",
            "",
            f"Changes affect {len(change_info['changed_files'])} file(s):"
        ]
        
        # List changed files (limit to 5)
        for file_path in change_info['changed_files'][:5]:
            body_parts.append(f"- {file_path}")
        
        if len(change_info['changed_files']) > 5:
            body_parts.append("- ... and more")
        
        body = "\\n".join(body_parts)
        
        return f"{subject}\\n\\n{body}"
    
    def execute_workflow(self, user_intent=None):
        """Execute the complete Git workflow."""
        print("🔍 Analyzing repository state...")
        
        if not self.check_repo_state():
            print("❌ Failed to analyze repository state")
            return False
        
        print(f"📍 Current branch: {self.current_branch}")
        print(f"📝 Uncommitted changes: {len(self.uncommitted_changes)} file(s)")
        
        change_info = self.analyze_changes()
        
        if change_info['type'] == 'empty':
            print("ℹ️ No changes detected in the working directory")
            user_response = input("Would you like to create an empty commit anyway? (y/N): ")
            if user_response.lower() != 'y':
                print("🛑 Workflow cancelled by user")
                return False
        
        # Generate branch name
        branch_name = self.generate_branch_name(change_info)
        if not branch_name:
            print("⚠️ Could not generate a meaningful branch name")
            branch_name = input("Enter a branch name: ").strip()
        
        print(f"🌿 Generated branch name: {branch_name}")
        
        # Generate commit message
        commit_msg = self.generate_commit_message(change_info)
        if not commit_msg:
            print("⚠️ Could not generate a commit message")
            commit_msg = input("Enter a commit message: ").strip()
        
        print(f"📦 Generated commit message: {commit_msg.split('\\n')[0][:50]}...")
        
        # Determine workflow based on current state
        if self.current_branch in ['main', 'master'] or 'protected' in self.current_branch:
            # On protected branch - create feature branch
            print(f"🔄 Creating new branch: {branch_name}")
            result = self.run_cmd(f"git checkout -b {branch_name}")
            if not result or result.returncode != 0:
                print(f"❌ Failed to create branch {branch_name}")
                return False
        elif self.current_branch != branch_name:
            # On different feature branch - ask user
            print(f"⚠️ You're on branch '{self.current_branch}' but changes suggest '{branch_name}'")
            response = input(f"Do you want to continue on current branch '{self.current_branch}'? (Y/n): ")
            if response.lower() == 'n':
                print(f"🔄 Switching to new branch: {branch_name}")
                result = self.run_cmd(f"git checkout -b {branch_name}")
                if not result or result.returncode != 0:
                    print(f"❌ Failed to create branch {branch_name}")
                    return False
        
        # Add changes
        print("📁 Adding changes to staging...")
        result = self.run_cmd("git add .")
        if not result or result.returncode != 0:
            print("❌ Failed to add changes")
            return False
        
        # Commit changes
        print("💾 Committing changes...")
        result = self.run_cmd(f'git commit -m "{commit_msg}"')
        if not result or result.returncode != 0:
            print("❌ Failed to commit changes")
            return False
        
        # Check if remote exists
        has_remote = bool(self.remotes)
        if not has_remote:
            print("⚠️ No remote repository configured")
            remote_url = input("Enter remote repository URL (or press Enter to skip): ").strip()
            if remote_url:
                result = self.run_cmd(f"git remote add origin {remote_url}")
                if not result or result.returncode != 0:
                    print("❌ Failed to add remote repository")
                    return False
                has_remote = True
        
        # Push changes
        if has_remote:
            print("📤 Pushing changes to remote...")
            # Use --set-upstream to establish tracking
            result = self.run_cmd(f"git push --set-upstream origin {self.current_branch}")
            if not result or result.returncode != 0:
                print(f"❌ Failed to push changes: {result.stderr if result else 'Unknown error'}")
                
                # Check if it's an authentication issue
                if result and ('authentication failed' in result.stderr.lower() or 
                              'permission denied' in result.stderr.lower()):
                    print("\\n💡 Authentication issue detected:")
                    print("   - Check your Git credentials")
                    print("   - Make sure you have push access to this repository")
                    print("   - Consider running: git config --global credential.helper store")
                
                return False
            print("✅ Changes pushed successfully!")
        else:
            print("⚠️ Skipping push (no remote configured)")
        
        # Create PR if gh CLI is available
        pr_created = False
        result = self.run_cmd("gh --version")
        if result and result.returncode == 0:
            print("🔗 GitHub CLI detected, creating pull request...")
            
            # Extract title from commit message
            title = commit_msg.split('\\n')[0].split(': ', 1)[1] if ': ' in commit_msg.split('\\n')[0] else commit_msg.split('\\n')[0]
            
            # Create PR
            result = self.run_cmd(f'gh pr create --title "{title}" --body "{commit_msg}"')
            if result and result.returncode == 0:
                print(f"✅ Pull request created: {result.stdout.strip()}")
                pr_created = True
            else:
                print(f"⚠️ Failed to create PR: {result.stderr if result else 'Unknown error'}")
        
        if not pr_created and has_remote:
            print("\\n💡 To create a pull request manually:")
            print(f"   Visit: {self._get_repo_url()}/compare/{self.current_branch}")
        
        print("\\n🎉 Git workflow completed successfully!")
        print(f"• Branch: {self.current_branch}")
        print(f"• Commit: {commit_msg.split('\\n')[0][:50]}...")
        if has_remote:
            print(f"• Remote: Pushed to origin/{self.current_branch}")
        
        return True
    
    def _get_repo_url(self):
        """Get the repository URL from remotes."""
        for remote in self.remotes:
            if 'origin' in remote and '(fetch)' in remote:
                url = remote.split()[1]
                # Convert SSH URL to HTTPS if needed
                if url.startswith('git@'):
                    # Convert git@host:org/repo.git to https://host/org/repo
                    host_and_path = url[4:].replace(':', '/')
                    if host_and_path.endswith('.git'):
                        host_and_path = host_and_path[:-4]
                    return f"https://{host_and_path}"
                return url
        return "[repository-url]"


def main():
    """Main entry point for the Git workflow agent."""
    print("🤖 Autonomous Git Workflow Agent")
    print("=" * 40)
    
    agent = GitWorkflowAgent()
    
    try:
        success = agent.execute_workflow()
        if success:
            print("\\n✅ Workflow executed successfully!")
        else:
            print("\\n❌ Workflow failed!")
            sys.exit(1)
    except KeyboardInterrupt:
        print("\\n\\n⚠️ Operation cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\\n❌ Unexpected error: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()