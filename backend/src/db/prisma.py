"""
Prisma database client singleton
"""
import os
from prisma import Prisma
from dotenv import load_dotenv

load_dotenv()

# Global prisma client instance
prisma: Prisma | None = None


async def get_prisma() -> Prisma:
    """Get or create Prisma client instance"""
    global prisma
    
    if prisma is None:
        prisma = Prisma()
        await prisma.connect()
    
    return prisma


async def disconnect_prisma():
    """Disconnect Prisma client"""
    global prisma
    
    if prisma is not None:
        await prisma.disconnect()
        prisma = None
