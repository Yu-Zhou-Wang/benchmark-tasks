"""Summarize one official Dream evaluation without modifying its logs or frames."""
import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re
import shutil
import statistics
import subprocess


EXPECTED = dict(model='dreamlabs/dream-1-beta', seed=0, max_steps=2400,
                object_jitter_m=0, control_hz=20, camera_pixels=[320, 176],
                rows_per_request=48, preroll_steps=32, epochs=1)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numeric(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def scalar(value):
    if numeric(value):
        return value
    if isinstance(value, dict):
        return next((value[k] for k in ('value', 'score', 'mean') if numeric(value.get(k))), None)
    return None


def trial_records(sample):
    """Only inspect known trial containers, never recursively walk transcripts."""
    records = [('samples[0]', sample)]
    for field in ('epochs', 'reduced'):
        value = sample.get(field)
        if isinstance(value, dict):
            records.append((f'samples[0].{field}', value))
            records.extend((f'samples[0].{field}.{k}', v) for k, v in list(value.items())[:20]
                           if isinstance(v, dict))
        elif isinstance(value, list):
            records.extend((f'samples[0].{field}[{i}]', v) for i, v in enumerate(value[:20])
                           if isinstance(v, dict))
    return records


def find_value(records, name):
    return next(((value[name], f'{path}.{name}') for path, value in records
                 if name in value and value[name] is not None), (None, None))


def timing(values):
    values = [float(v) for v in values if numeric(v)]
    return None if not values else dict(count=len(values), minimum=min(values),
                                        median=statistics.median(values), maximum=max(values))


def video_info(path, expected_frames):
    result = {'file': str(path), 'expected_frames': expected_frames}
    try:
        executable = shutil.which('ffprobe')
        if executable:
            probe = subprocess.run([executable, '-v', 'error', '-select_streams', 'v:0',
                '-count_frames', '-show_entries',
                'stream=codec_name,width,height,avg_frame_rate,nb_read_frames', '-of', 'json', str(path)],
                capture_output=True, text=True, timeout=180, check=True)
            stream = json.loads(probe.stdout)['streams'][0]
            result.update(method='ffprobe_count_frames', codec=stream.get('codec_name'),
                width=stream.get('width'), height=stream.get('height'),
                fps=float(Fraction(stream['avg_frame_rate'])),
                frame_count=int(stream['nb_read_frames']), full_scan=True)
        else:
            import imageio.v2 as imageio
            count = 0
            shapes = set()
            with imageio.get_reader(path) as reader:
                metadata = reader.get_meta_data()
                for frame in reader:
                    count += 1
                    shapes.add(tuple(frame.shape[:2]))
                    if count > 100000:
                        raise ValueError('Video exceeds bounded 100000-frame scan.')
            if len(shapes) != 1:
                raise ValueError(f'Unexpected decoded image sizes: {sorted(shapes)}')
            height, width = next(iter(shapes))
            result.update(method='imageio_full_decode', codec=metadata.get('codec'),
                          width=width, height=height, fps=metadata.get('fps'),
                          frame_count=count, full_scan=True)
        result['verified'] = bool(expected_frames is not None
            and result['frame_count'] == expected_frames
            and result['width'] == 320 and result['height'] == 176
            and numeric(result['fps']) and abs(result['fps'] - 20.) < 1e-6)
    except Exception as error:
        result.update(verified=False, error=f'{type(error).__name__}: {error}')
    return result


def save_stages(run, frames_dir, cameras):
    """Save exact raw arrays as PNG; no resizing, annotation, or source writes."""
    import numpy as np
    from PIL import Image
    candidate = Path(frames_dir) if isinstance(frames_dir, str) else run / 'eval/frames'
    if not candidate.is_absolute():
        candidate = run / candidate
    if not candidate.is_dir() or not candidate.resolve().is_relative_to(run):
        candidate = run / 'eval/frames'
    paths = list(candidate.rglob('*.npy')) if candidate.is_dir() else []
    natural = lambda p: [int(s) if s.isdigit() else s for s in re.split(r'(\d+)', p.as_posix())]
    saved = []
    for camera in cameras:
        files = sorted((p for p in paths if camera in p.name or camera in p.parts), key=natural)
        if not files:
            continue
        for label, index in [('initial', 0), ('middle', (len(files) - 1) // 2), ('final', len(files) - 1)]:
            source = files[index]
            pixels = np.load(source, allow_pickle=False)
            if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
                saved.append(dict(camera=camera, stage=label, source=str(source.relative_to(run)),
                                  error=f'Unsupported raw image shape/dtype: {pixels.shape}/{pixels.dtype}'))
                continue
            destination = run / 'stages' / f'{label}_{camera}.png'
            destination.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(pixels).save(destination)
            saved.append(dict(camera=camera, stage=label, raw_frame_index=index,
                raw_frame_count=len(files), source=str(source.relative_to(run)),
                image=str(destination.relative_to(run))))
    return saved


def collect(run, stages=True):
    run = Path(run).resolve()
    manifest_path = run / 'run_manifest.json'
    manifest = read_json(manifest_path)
    candidates, issues, missing = [], [], []
    for path in sorted((run / 'eval').glob('*.json')):
        if path.name.endswith('.live.json'):
            continue
        try:
            log = read_json(path)
            if isinstance(log, dict) and isinstance(log.get('eval'), dict):
                candidates.append((path, log))
        except (OSError, ValueError) as error:
            issues.append(f'Could not read {path.name}: {error}')
    if len(candidates) != 1:
        raise ValueError(f'Expected exactly one final official EvalLog, found {len(candidates)}.')
    path, log = candidates[0]
    evaluation, results, stats = (log.get(name) or {} for name in ('eval', 'results', 'stats'))
    samples = log.get('samples') or []
    sample = samples[0] if samples and isinstance(samples[0], dict) else {}
    records = trial_records(sample)
    instruction, instruction_source = find_value(records, 'instruction')
    terminations, termination_source = find_value(records, 'termination_reasons')
    transcripts, transcript_source = find_value(records, 'policy_transcripts')
    transcript = transcripts[0] if isinstance(transcripts, list) and transcripts else None
    if isinstance(transcript, dict) and 'model' not in transcript:
        for field in ('transcript', 'data', 'content'):
            if isinstance(transcript.get(field), dict):
                transcript = transcript[field]
                transcript_source += f'[0].{field}'
                break
    if not isinstance(transcript, dict):
        transcript = {}
    requests = transcript.get('requests')
    request_rows = requests if isinstance(requests, list) else []
    request_rows = [r for r in request_rows if isinstance(r, dict)]
    transcript_fields = ('model', 'control_hz', 'rows_per_request', 'preroll_steps', 'dry_run')
    missing.extend(f'transcript.{key}' for key in (*transcript_fields, 'requests') if key not in transcript)
    if instruction is None:
        missing.append('trial.instruction')
    if terminations is None:
        missing.append('trial.termination_reasons')
    settings = manifest.get('settings') or {}
    mismatches = []
    def require(label, actual, expected):
        if actual is None:
            missing.append(label)
        elif actual != expected:
            mismatches.append(dict(field=label, expected=expected, observed=actual))
    for key, expected in EXPECTED.items():
        require('manifest.' + key, settings.get(key), expected)
    for key, expected in dict(policy='dream', seed=0, max_steps=2400).items():
        require('eval.' + key, evaluation.get(key), expected)
    observed_hz = (evaluation.get('embodiment_info') or {}).get('control_hz')
    require('eval.embodiment_info.control_hz', observed_hz, 20)
    for key, expected in dict(model=EXPECTED['model'], control_hz=20, rows_per_request=48,
                              preroll_steps=32, dry_run=False).items():
        require('transcript.' + key, transcript.get(key), expected)
    checkpoint = evaluation.get('policy_checkpoint')
    if checkpoint is None:
        missing.append('eval.policy_checkpoint')
    elif isinstance(checkpoint, str):
        require('eval.policy_checkpoint', checkpoint, EXPECTED['model'])
    if instruction is not None:
        require('trial.instruction', instruction, manifest.get('instruction'))
    total_trials, errored_trials = results.get('total_trials'), results.get('errored_trials')
    require('results.total_trials', total_trials, 1)
    require('results.errored_trials', errored_trials, 0)
    require('samples.count', len(samples), 1)
    score = scalar((results.get('metrics') or {}).get('task_score'))
    if score is None:
        missing.append('results.metrics.task_score (numeric)')
    before, after = manifest.get('source_before'), manifest.get('source_after')
    unchanged = bool(isinstance(before, dict) and before.get('files') and before == after
                     and manifest.get('source_unchanged') is True)
    if not unchanged:
        issues.append('Source fingerprints are absent, changed, or not confirmed unchanged.')
    steps = stats.get('total_steps')
    expected_frames = int(steps) + 1 if numeric(steps) and steps >= 0 and int(steps) == steps else None
    videos = [video_info(video, expected_frames) for video in sorted((run / 'video').glob('*.mp4'))]
    for video in videos:
        video['file'] = str(Path(video['file']).relative_to(run))
    cameras = settings.get('cameras') or ['top_cam', 'left_cam', 'right_cam']
    videos_verified = len(videos) == len(cameras) and all(v['verified'] for v in videos)
    if not videos_verified:
        issues.append('Exported videos do not all meet the expected count, dimensions, and frame rate.')
    stage_images = []
    if stages:
        try:
            stage_images = save_stages(run, stats.get('frames_dir'), cameras)
        except Exception as error:
            issues.append(f'Stage PNG extraction unavailable: {type(error).__name__}: {error}')
    sessions = {str(r['session_id']) for r in request_rows if r.get('session_id') is not None}
    execution_success = manifest.get('returncode') == 0 and total_trials == 1 and errored_trials == 0
    return {
        'schema_version': 1, 'execution_success': execution_success,
        'task_score': score, 'task_success': None if score is None else score == 1.0,
        'task_success_basis': 'Official task_score equals 1.0; execution completion does not require this.',
        'configuration_verified': not mismatches and not missing,
        'expected_configuration': EXPECTED, 'configuration_mismatches': mismatches,
        'missing_fields': sorted(set(missing)), 'source_unchanged': unchanged,
        'manifest': dict(file='run_manifest.json', sha256=digest(manifest_path),
                         returncode=manifest.get('returncode'), status=manifest.get('status'),
                         wall_seconds=manifest.get('wall_seconds'), versions=manifest.get('versions'),
                         hosted_model_seeded=settings.get('hosted_model_seeded')),
        'official_log': dict(file=str(path.relative_to(run)), sha256=digest(path),
            version=log.get('version'), policy=evaluation.get('policy'), policy_checkpoint=checkpoint,
            seed=evaluation.get('seed'), max_steps=evaluation.get('max_steps'), control_hz=observed_hz,
            total_trials=total_trials, errored_trials=errored_trials, sample_count=len(samples),
            total_steps=steps, duration_s=stats.get('duration_s'), frames_dir=stats.get('frames_dir'),
            epochs=len(sample['epochs']) if isinstance(sample.get('epochs'), (dict, list)) else None,
            reduced_present='reduced' in sample, instruction=instruction, instruction_source=instruction_source,
            termination_reasons=terminations, termination_source=termination_source),
        'transcript': dict(source=transcript_source, **{k: transcript.get(k) for k in transcript_fields},
            request_count=len(requests) if isinstance(requests, list) else None,
            request_steps=[request_rows[0].get('step'), request_rows[-1].get('step')] if request_rows else None,
            rtt_seconds=timing([r.get('rtt_s') for r in request_rows]),
            inference_ms=timing([r.get('infer_ms') for r in request_rows]),
            attempts=timing([r.get('attempts') for r in request_rows]),
            response_rows=timing([r.get('rows') for r in request_rows]),
            unique_session_count=len(sessions)),
        'videos_verified': videos_verified, 'videos': videos, 'stage_images': stage_images, 'issues': issues,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--no-stages', action='store_true')
    args = parser.parse_args()
    result = collect(args.run_directory, stages=not args.no_stages)
    destination = args.run_directory.resolve() / 'result.json'
    destination.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({key: result[key] for key in ('execution_success', 'task_score', 'task_success',
        'configuration_verified', 'source_unchanged', 'videos_verified', 'missing_fields', 'issues')}, indent=2))
