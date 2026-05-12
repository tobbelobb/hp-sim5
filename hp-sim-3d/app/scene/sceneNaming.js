export function scopedKey(namespace, relativeName) {
    const key = relativeName || '';
    return namespace ? `${namespace}::${key}` : key;
}

export function scopedKeyFromPath(namespace, sceneRootPath, fullPath) {
    if (typeof fullPath !== 'string') {
        return scopedKey(namespace, fullPath);
    }
    let relative = fullPath;
    if (sceneRootPath && fullPath.startsWith(sceneRootPath)) {
        relative = fullPath.slice(sceneRootPath.length);
        if (relative.startsWith('/')) {
            relative = relative.slice(1);
        }
    } else if (fullPath.startsWith('/')) {
        relative = fullPath.slice(1);
    }
    const sanitized = relative.replace(/\//g, '::');
    return scopedKey(namespace, sanitized || relative);
}
