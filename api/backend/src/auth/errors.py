"""
Authentication error types and handling
"""


class AuthError(Exception):
    """Base authentication error"""
    
    def __init__(self, message: str, code: str = "AUTH_ERROR"):
        self.message = message
        self.code = code
        super().__init__(self.message)


class UserExistsError(AuthError):
    """Email already registered"""
    
    def __init__(self, message: str = "User with this email already exists"):
        super().__init__(message, code="USER_EXISTS")


class InvalidCredentialsError(AuthError):
    """Invalid email or password"""
    
    def __init__(self, message: str = "Invalid email or password"):
        super().__init__(message, code="INVALID_CREDENTIALS")


class InvalidInputError(AuthError):
    """Invalid input data"""
    
    def __init__(self, message: str = "Invalid input"):
        super().__init__(message, code="INVALID_INPUT")


class ServerError(AuthError):
    """Internal server error"""
    
    def __init__(self, message: str = "Internal server error"):
        super().__init__(message, code="SERVER_ERROR")


class SessionExpiredError(AuthError):
    """Session has expired"""
    
    def __init__(self, message: str = "Session has expired"):
        super().__init__(message, code="SESSION_EXPIRED")


class InvalidTokenError(AuthError):
    """Invalid or expired reset token"""
    
    def __init__(self, message: str = "Invalid or expired reset token"):
        super().__init__(message, code="INVALID_TOKEN")
