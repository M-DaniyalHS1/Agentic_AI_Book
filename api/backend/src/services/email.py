"""
Email service for sending password reset and other authentication emails
Uses Gmail SMTP with app password
"""
import os
import aiosmtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from dotenv import load_dotenv

load_dotenv()


class EmailService:
    """Email service for sending authentication-related emails"""
    
    def __init__(self):
        self.smtp_host = os.getenv("SMTP_HOST", "smtp.gmail.com")
        self.smtp_port = int(os.getenv("SMTP_PORT", "587"))
        self.smtp_username = os.getenv("SMTP_USERNAME", "")
        self.smtp_password = os.getenv("SMTP_PASSWORD", "")
        self.from_name = os.getenv("SMTP_FROM_NAME", "Agent Book Factory")
        self.from_email = os.getenv("SMTP_FROM_EMAIL", "noreply@agent-book-factory.vercel.app")
    
    async def send_password_reset_email(self, to_email: str, reset_link: str) -> bool:
        """
        Send password reset email
        
        Args:
            to_email: Recipient email address
            reset_link: Password reset link with token
            
        Returns:
            True if email sent successfully, False otherwise
        """
        if not self.smtp_username or not self.smtp_password:
            print("⚠ SMTP credentials not configured - skipping email send (development mode)")
            print(f"Reset link for {to_email}: {reset_link}")
            return False
        
        # Create message
        message = MIMEMultipart("alternative")
        message["Subject"] = "Password Reset Request - Agent Book Factory"
        message["From"] = f"{self.from_name} <{self.from_email}>"
        message["To"] = to_email
        
        # Create plain text version
        text = f"""
Hello,

You have requested to reset your password for Agent Book Factory.

Click the link below to reset your password:
{reset_link}

This link will expire in 1 hour.

If you did not request a password reset, please ignore this email.

Best regards,
Agent Book Factory Team
"""
        
        # Create HTML version
        html = f"""
<!DOCTYPE html>
<html>
<head>
    <style>
        body {{ font-family: Arial, sans-serif; line-height: 1.6; color: #333; }}
        .container {{ max-width: 600px; margin: 0 auto; padding: 20px; }}
        .button {{ 
            display: inline-block; 
            padding: 12px 24px; 
            background-color: #4F46E5; 
            color: white; 
            text-decoration: none; 
            border-radius: 6px;
            margin: 20px 0;
        }}
        .footer {{ 
            margin-top: 30px; 
            padding-top: 20px; 
            border-top: 1px solid #ddd; 
            font-size: 12px; 
            color: #666;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h2>Password Reset Request</h2>
        <p>Hello,</p>
        <p>You have requested to reset your password for Agent Book Factory.</p>
        <p>Click the button below to reset your password:</p>
        <p>
            <a href="{reset_link}" class="button">Reset Password</a>
        </p>
        <p>Or copy and paste this link into your browser:</p>
        <p style="word-break: break-all; color: #666;">{reset_link}</p>
        <p><strong>This link will expire in 1 hour.</strong></p>
        <p>If you did not request a password reset, please ignore this email.</p>
        <div class="footer">
            <p>Best regards,<br>Agent Book Factory Team</p>
        </div>
    </div>
</body>
</html>
"""
        
        # Attach both versions
        message.attach(MIMEText(text, "plain"))
        message.attach(MIMEText(html, "html"))
        
        try:
            # Send email
            await aiosmtplib.send(
                message,
                hostname=self.smtp_host,
                port=self.smtp_port,
                start_tls=True,
                username=self.smtp_username,
                password=self.smtp_password,
            )
            print(f"✓ Password reset email sent to {to_email}")
            return True
        except Exception as e:
            print(f"✗ Failed to send email to {to_email}: {e}")
            return False
    
    async def send_welcome_email(self, to_email: str) -> bool:
        """
        Send welcome email after signup
        
        Args:
            to_email: Recipient email address
            
        Returns:
            True if email sent successfully, False otherwise
        """
        if not self.smtp_username or not self.smtp_password:
            print("⚠ SMTP credentials not configured - skipping welcome email (development mode)")
            return False
        
        # Create message
        message = MIMEMultipart("alternative")
        message["Subject"] = "Welcome to Agent Book Factory!"
        message["From"] = f"{self.from_name} <{self.from_email}>"
        message["To"] = to_email
        
        # Create plain text version
        text = f"""
Welcome to Agent Book Factory!

Your account has been created successfully.

Get started by exploring our AI-powered robotics textbook and personalized learning resources.

Best regards,
Agent Book Factory Team
"""
        
        # Create HTML version
        html = f"""
<!DOCTYPE html>
<html>
<head>
    <style>
        body {{ font-family: Arial, sans-serif; line-height: 1.6; color: #333; }}
        .container {{ max-width: 600px; margin: 0 auto; padding: 20px; }}
        .button {{ 
            display: inline-block; 
            padding: 12px 24px; 
            background-color: #4F46E5; 
            color: white; 
            text-decoration: none; 
            border-radius: 6px;
        }}
        .footer {{ 
            margin-top: 30px; 
            padding-top: 20px; 
            border-top: 1px solid #ddd; 
            font-size: 12px; 
            color: #666;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h2>Welcome to Agent Book Factory! 🎉</h2>
        <p>Your account has been created successfully.</p>
        <p>Get started by exploring our AI-powered robotics textbook and personalized learning resources.</p>
        <p>
            <a href="https://agent-book-factory.vercel.app/dashboard" class="button">Go to Dashboard</a>
        </p>
        <div class="footer">
            <p>Best regards,<br>Agent Book Factory Team</p>
        </div>
    </div>
</body>
</html>
"""
        
        message.attach(MIMEText(text, "plain"))
        message.attach(MIMEText(html, "html"))
        
        try:
            await aiosmtplib.send(
                message,
                hostname=self.smtp_host,
                port=self.smtp_port,
                start_tls=True,
                username=self.smtp_username,
                password=self.smtp_password,
            )
            print(f"✓ Welcome email sent to {to_email}")
            return True
        except Exception as e:
            print(f"✗ Failed to send welcome email to {to_email}: {e}")
            return False


# Singleton instance
email_service = EmailService()
