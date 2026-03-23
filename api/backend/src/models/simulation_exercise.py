"""
Simulation Exercise and Assessment Models
"""
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, Float, Boolean, JSON, Enum
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import enum
from ..database import Base


class SimulationType(str, enum.Enum):
    """Types of simulations"""
    GAZEBO = "gazebo"
    UNITY = "unity"
    ISAAC_SIM = "isaac_sim"
    ROS2 = "ros2"


class SimulationExercise(Base):
    """Simulation Exercise - Hands-on robotics simulations"""
    __tablename__ = "simulation_exercises"

    id = Column(Integer, primary_key=True, index=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=False)
    title = Column(String, nullable=False)
    slug = Column(String, nullable=False)
    description = Column(Text)
    simulation_type = Column(String)  # gazebo, unity, isaac_sim, ros2
    ros_package = Column(String)  # ROS package name
    launch_file = Column(String)  # ROS launch file path
    world_file = Column(String)  # Simulation world file
    instructions = Column(Text)  # Step-by-step instructions
    prerequisites = Column(JSON)  # Required knowledge/skills
    expected_outcome = Column(Text)
    difficulty_level = Column(String)  # beginner, intermediate, advanced
    estimated_time_minutes = Column(Integer)
    is_runnable = Column(Boolean, default=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationships
    chapter = relationship("Chapter")


class AssessmentItem(Base):
    """Assessment Item - Quiz questions and exercises"""
    __tablename__ = "assessment_items"

    id = Column(Integer, primary_key=True, index=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=False)
    question = Column(Text, nullable=False)
    question_type = Column(String)  # multiple_choice, true_false, short_answer, code
    options = Column(JSON)  # For multiple choice: list of options
    correct_answer = Column(Text)  # Can be text or JSON for structured answers
    explanation = Column(Text)  # Explanation of the correct answer
    points = Column(Integer, default=1)
    difficulty = Column(String)  # easy, medium, hard
    tags = Column(JSON)  # Topics/skills this assesses
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    # Relationships
    chapter = relationship("Chapter")


class StudentAssessment(Base):
    """Student Assessment - Tracks student assessment attempts"""
    __tablename__ = "student_assessments"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(Integer, ForeignKey("student_sessions.id"), nullable=False)
    assessment_item_id = Column(Integer, ForeignKey("assessment_items.id"), nullable=False)
    student_answer = Column(Text)
    is_correct = Column(Boolean)
    points_earned = Column(Integer, default=0)
    submitted_at = Column(DateTime(timezone=True), server_default=func.now())

    # Relationships
    session = relationship("StudentSession")
    assessment_item = relationship("AssessmentItem")
