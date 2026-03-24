"""Initial database schema - modules, chapters, sections, sessions, AI responses, simulations, assessments

Revision ID: 001_initial_schema
Revises: 
Create Date: 2026-03-07

"""
from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = '001_initial_schema'
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    # Create modules table
    op.create_table(
        'modules',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('slug', sa.String(), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('sidebar_position', sa.Integer(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_modules_id'), 'modules', ['id'], unique=False)
    op.create_index(op.f('ix_modules_slug'), 'modules', ['slug'], unique=True)

    # Create chapters table
    op.create_table(
        'chapters',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('module_id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('slug', sa.String(), nullable=False),
        sa.Column('content', sa.Text(), nullable=True),
        sa.Column('learning_objectives', sa.JSON(), nullable=True),
        sa.Column('prerequisites', sa.JSON(), nullable=True),
        sa.Column('key_takeaways', sa.JSON(), nullable=True),
        sa.Column('sidebar_position', sa.Integer(), nullable=True),
        sa.Column('word_count', sa.Integer(), nullable=True),
        sa.Column('git_sha', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.ForeignKeyConstraint(['module_id'], ['modules.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_chapters_id'), 'chapters', ['id'], unique=False)

    # Create sections table
    op.create_table(
        'sections',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('slug', sa.String(), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('content_html', sa.Text(), nullable=True),
        sa.Column('sidebar_position', sa.Integer(), nullable=True),
        sa.Column('word_count', sa.Integer(), nullable=True),
        sa.Column('git_sha', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_sections_id'), 'sections', ['id'], unique=False)

    # Create student_sessions table
    op.create_table(
        'student_sessions',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('session_id', sa.String(), nullable=False),
        sa.Column('user_id', sa.String(), nullable=True),
        sa.Column('started_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('last_active', sa.DateTime(timezone=True), nullable=True),
        sa.Column('current_module', sa.String(), nullable=True),
        sa.Column('current_chapter', sa.String(), nullable=True),
        sa.Column('metadata', sa.JSON(), nullable=True),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('session_id')
    )
    op.create_index(op.f('ix_student_sessions_id'), 'student_sessions', ['id'], unique=False)
    op.create_index(op.f('ix_student_sessions_session_id'), 'student_sessions', ['session_id'], unique=True)

    # Create ai_queries table
    op.create_table(
        'ai_queries',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('session_id', sa.Integer(), nullable=False),
        sa.Column('query_text', sa.Text(), nullable=False),
        sa.Column('selected_text', sa.Text(), nullable=True),
        sa.Column('query_type', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['session_id'], ['student_sessions.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_ai_queries_id'), 'ai_queries', ['id'], unique=False)

    # Create ai_responses table
    op.create_table(
        'ai_responses',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('query_id', sa.Integer(), nullable=False),
        sa.Column('answer', sa.Text(), nullable=False),
        sa.Column('confidence_score', sa.Float(), nullable=True),
        sa.Column('is_fallback', sa.Boolean(), nullable=True),
        sa.Column('response_time_ms', sa.Integer(), nullable=True),
        sa.Column('model_used', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['query_id'], ['ai_queries.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_ai_responses_id'), 'ai_responses', ['id'], unique=False)

    # Create citations table
    op.create_table(
        'citations',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('response_id', sa.Integer(), nullable=False),
        sa.Column('module_slug', sa.String(), nullable=True),
        sa.Column('chapter_slug', sa.String(), nullable=True),
        sa.Column('section_slug', sa.String(), nullable=True),
        sa.Column('content_snippet', sa.Text(), nullable=True),
        sa.Column('similarity_score', sa.Float(), nullable=True),
        sa.Column('citation_order', sa.Integer(), nullable=True),
        sa.ForeignKeyConstraint(['response_id'], ['ai_responses.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_citations_id'), 'citations', ['id'], unique=False)

    # Create simulation_exercises table
    op.create_table(
        'simulation_exercises',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('title', sa.String(), nullable=False),
        sa.Column('slug', sa.String(), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('simulation_type', sa.String(), nullable=True),
        sa.Column('ros_package', sa.String(), nullable=True),
        sa.Column('launch_file', sa.String(), nullable=True),
        sa.Column('world_file', sa.String(), nullable=True),
        sa.Column('instructions', sa.Text(), nullable=True),
        sa.Column('prerequisites', sa.JSON(), nullable=True),
        sa.Column('expected_outcome', sa.Text(), nullable=True),
        sa.Column('difficulty_level', sa.String(), nullable=True),
        sa.Column('estimated_time_minutes', sa.Integer(), nullable=True),
        sa.Column('is_runnable', sa.Boolean(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_simulation_exercises_id'), 'simulation_exercises', ['id'], unique=False)

    # Create assessment_items table
    op.create_table(
        'assessment_items',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('chapter_id', sa.Integer(), nullable=False),
        sa.Column('question', sa.Text(), nullable=False),
        sa.Column('question_type', sa.String(), nullable=True),
        sa.Column('options', sa.JSON(), nullable=True),
        sa.Column('correct_answer', sa.Text(), nullable=True),
        sa.Column('explanation', sa.Text(), nullable=True),
        sa.Column('points', sa.Integer(), nullable=True),
        sa.Column('difficulty', sa.String(), nullable=True),
        sa.Column('tags', sa.JSON(), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['chapter_id'], ['chapters.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_assessment_items_id'), 'assessment_items', ['id'], unique=False)

    # Create student_assessments table
    op.create_table(
        'student_assessments',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('session_id', sa.Integer(), nullable=False),
        sa.Column('assessment_item_id', sa.Integer(), nullable=False),
        sa.Column('student_answer', sa.Text(), nullable=True),
        sa.Column('is_correct', sa.Boolean(), nullable=True),
        sa.Column('points_earned', sa.Integer(), nullable=True),
        sa.Column('submitted_at', sa.DateTime(timezone=True), server_default=sa.text('now()'), nullable=True),
        sa.ForeignKeyConstraint(['assessment_item_id'], ['assessment_items.id'], ),
        sa.ForeignKeyConstraint(['session_id'], ['student_sessions.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_student_assessments_id'), 'student_assessments', ['id'], unique=False)


def downgrade() -> None:
    op.drop_table('student_assessments')
    op.drop_table('assessment_items')
    op.drop_table('simulation_exercises')
    op.drop_table('citations')
    op.drop_table('ai_responses')
    op.drop_table('ai_queries')
    op.drop_table('student_sessions')
    op.drop_table('sections')
    op.drop_table('chapters')
    op.drop_table('modules')
