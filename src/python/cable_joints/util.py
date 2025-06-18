def is_attachment(value):
    """Checks if a link type is an attachment point."""
    return value in ['attachment', 'hybrid-attachment', 'pinhole']

def is_rolling(value):
    """Checks if a link type is a rolling contact."""
    return value in ['rolling', 'hybrid']

def is_hybrid(value):
    """Checks if a link type is a hybrid or hybrid-attachment."""
    return value in ['hybrid', 'hybrid-attachment']

def effective_cw(path, link_index, travelling_from_circle):
    """
    Determines the effective clockwise direction for tangent calculations.
    This is a special case for the first link in a path when calculating
    the tangent from that link (which is a circle).
    """
    if link_index == 0 and travelling_from_circle:
        return not path.cw[link_index]
    return path.cw[link_index]
