def is_in(state, goals):
    return any([state is goal for goal in goals])