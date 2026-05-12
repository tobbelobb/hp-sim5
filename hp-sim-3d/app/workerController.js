export const KLIPPER_UPLOAD_PIPELINE = 'player';

export function shouldUseRawKlipperUploadPipeline(pipeline = KLIPPER_UPLOAD_PIPELINE) {
  return pipeline === 'raw';
}

