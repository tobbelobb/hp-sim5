"""Conversation policies shared by local and current-chat managers."""
import json
from pathlib import Path


def manager_policy(*, grill_me=False, ask_questions=False):
    policy = (
        '\nHuman communication policy:\n'
        'Use request_user_input or AskUserQuestion when available; otherwise ask in chat '
        'and end the turn to wait for an answer. Never treat silence, a dismissed question '
        'or a tool timeout as agreement. Human steering can change this policy at any time.\n'
    )
    if ask_questions:
        policy += (
            'During research, ask the human when new evidence changes the intended outcome, '
            'scope, tradeoffs, acceptance criteria or authority, or reveals a consequential '
            'assumption only they can resolve. Explain the decision and recommend an answer. '
            'Continue independent work while waiting, but hold dependent work. '
            'Resolve facts from the repository and routine technical choices yourself.\n'
        )
    else:
        policy += (
            'After any startup interview, work autonomously within the recorded scope. '
            'Do not routinely solicit human answers; record reasonable assumptions. '
            'If missing authority or an essential human decision blocks safe progress, '
            'report that blocker rather than inventing an answer.\n'
        )
    if grill_me:
        policy += (
            '\nStartup grill session: interview the human before commissioning or dispatching '
            'new work. Inspect the overview, catalog and relevant history first; look up '
            'available facts instead of asking the human to retrieve them. Work through '
            'dependent decisions in rounds: ask a few focused questions whose prerequisites '
            'are settled, offer recommendations with reasons, then wait for real answers. '
            'Follow up on contradictions and vague outcomes; do not dump a fixed questionnaire.\n'
            'Establish why this research is worth doing now, the practical decision it serves, '
            'history and failed attempts, intended users, expected outcomes (including whether '
            'a tested implementation is required), measurable success and failure, baselines, '
            'constraints/non-goals, total research/review/check budgets, and integration/push '
            'authority. Challenge the premise: what would make us abandon it, what is the '
            'cheapest falsifying experiment, and could existing evidence settle it?\n'
            'Explain that the human may stop the interview at any time. Stop when they ask '
            'or when consequential decisions are sufficiently clear; do not require another '
            'approval ceremony. Distinguish ending the interview from stopping the program. '
            'Summarize answers, unresolved assumptions, delivery criteria and the first cheap '
            'test in research.md and preserve human answers using steering and decisions. '
            'Then proceed within established authority, or report the specific blocker.\n'
        )
    else:
        policy += 'No startup grill session was requested.\n'
    return policy


def worker_policy(session):
    return (
        '\nYour clarification contact is the research manager, never the human. '
        'Do not use human question tools. Resolve repository facts yourself. For a blocking '
        'manager decision, atomically write ' + str(session / 'manager-question.json') +
        ' as {"id":"a unique question id","question":"question, context and recommended answer"}. '
        'The manager polls worker_status and atomically writes ' + str(session / 'manager-answer.json') +
        ' as {"id":"the same question id","answer":"manager decision"}. '
        'Accept only an answer matching your current id. Ask one question at a time; keep '
        'unique ids so old answers cannot answer new questions. Continue independent work '
        'while waiting; poll at intervals of a few seconds, within the existing wall deadline. '
        'Never extend budgets or assume silence grants authority. If no answer arrives, '
        'retain the question and report blocked/partial evidence. Record answers in research.md.\n'
    )


def pending_question(session):
    """Read a small worker mailbox without ingesting logs or partial writes."""
    def read(name):
        with (Path(session) / name).open('rb') as stream:
            raw = stream.read(16385)
        if len(raw) > 16384:
            raise ValueError('Clarification exceeds 16 KiB')
        return json.loads(raw)

    try:
        question = read('manager-question.json')
        if not isinstance(question, dict) or not all(
                isinstance(question.get(key), str) and question[key].strip() for key in ('id', 'question')):
            return None
        try:
            answer = read('manager-answer.json')
        except (OSError, ValueError):
            answer = None
        if (isinstance(answer, dict) and answer.get('id') == question['id'] and
                isinstance(answer.get('answer'), str) and answer['answer'].strip()):
            return None
        return question
    except (OSError, ValueError):
        return None
