#!/usr/bin/env python3
"""
Tests for the Git Workflow Agent
"""

import unittest
import tempfile
import os
import subprocess
from unittest.mock import Mock, patch
from git_workflow_agent import GitWorkflowAgent


class TestGitWorkflowAgent(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures."""
        self.agent = GitWorkflowAgent()
        
    def test_branch_name_generation(self):
        """Test that branch names are generated correctly."""
        # Test with feature-type changes
        change_info = {
            'type': 'feat',
            'scope': 'api',
            'changed_files': ['src/api/users.js', 'src/models/userModel.js']
        }
        
        branch_name = self.agent.generate_branch_name(change_info)
        self.assertIsNotNone(branch_name)
        self.assertTrue(branch_name.startswith('feature/'))
        
        # Test with fix-type changes
        change_info = {
            'type': 'fix',
            'scope': 'auth',
            'changed_files': ['src/auth/login.js']
        }
        
        branch_name = self.agent.generate_branch_name(change_info)
        self.assertIsNotNone(branch_name)
        self.assertTrue(branch_name.startswith('fix/'))
        
    def test_commit_message_generation(self):
        """Test that commit messages are generated correctly."""
        change_info = {
            'type': 'feat',
            'scope': 'api',
            'changed_files': ['src/api/users.js', 'src/models/userModel.js'],
            'diff_stat': '2 files changed, 15 insertions(+), 3 deletions(-)'
        }
        
        commit_msg = self.agent.generate_commit_message(change_info)
        self.assertIsNotNone(commit_msg)
        self.assertIn('feat(', commit_msg)
        self.assertIn('users', commit_msg.lower())
        
    @patch('subprocess.run')
    def test_run_cmd_success(self, mock_subprocess):
        """Test that run_cmd works with successful command."""
        mock_result = Mock()
        mock_result.returncode = 0
        mock_result.stdout = "success output"
        mock_subprocess.return_value = mock_result
        
        result = self.agent.run_cmd("echo test")
        self.assertIsNotNone(result)
        self.assertEqual(result.stdout, "success output")
        
    @patch('subprocess.run')
    def test_run_cmd_failure(self, mock_subprocess):
        """Test that run_cmd handles failed command."""
        mock_result = Mock()
        mock_result.returncode = 1
        mock_result.stdout = ""
        mock_result.stderr = "error message"
        mock_subprocess.return_value = mock_result
        
        result = self.agent.run_cmd("invalid_command")
        self.assertIsNotNone(result)
        self.assertEqual(result.returncode, 1)
        
    def test_empty_change_info(self):
        """Test handling of empty change info."""
        change_info = {
            'type': 'empty',
            'description': 'No changes detected'
        }
        
        branch_name = self.agent.generate_branch_name(change_info)
        self.assertIsNone(branch_name)
        
        commit_msg = self.agent.generate_commit_message(change_info)
        self.assertIsNone(commit_msg)


if __name__ == '__main__':
    unittest.main()