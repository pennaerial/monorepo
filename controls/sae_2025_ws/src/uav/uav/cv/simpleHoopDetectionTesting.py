import cv2
import numpy as np
import argparse
import os
import glob
import json
from hoopDetectorAlgorithm import (
    find_nearest_hoop_pose, draw_hoop_pose_overlay, get_default_K, 
    RING_RAD_M_DEFAULT
)

# --- Directory Management ---
def get_next_run_dir(base_dir="SimpleAlgTestingOnRealSim"):
    """Finds the next run_XXX directory."""
    os.makedirs(base_dir, exist_ok=True)
    existing_runs = glob.glob(os.path.join(base_dir, "run*"))
    
    max_idx = 0
    for run in existing_runs:
        try:
            name = os.path.basename(run)
            if name.startswith("run"):
                idx = int(name[3:])
                max_idx = max(max_idx, idx)
        except ValueError:
            continue
            
    next_idx = max_idx + 1
    run_dir = os.path.join(base_dir, f"run{next_idx}")
    os.makedirs(run_dir, exist_ok=True)
    return run_dir

# --- Main Processing ---
def process_video_and_save_debug(input_path, output_dir, K, ring_radii_m):
    """
    Process video with debugging information.
    """
    
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {input_path}")
        return

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    
    # Initialize output writer
    final_output_path = os.path.join(output_dir, "00_hoop_pose_overlay.mp4")
    out_final = cv2.VideoWriter(final_output_path, fourcc, fps, (frame_width, frame_height))
    
    # Writers for the *kept* intermediate debug files
    intermediate_writers = {}
    intermediate_file_names = ['02_Binary_Mask', '04_Ellipse_Candidates', '06_Sampled_Points']
    
    # Statistics (for console summary only)
    stats = {
        'total_frames': 0,
        'detected_frames': 0,
        'radius_usage': {r: 0 for r in ring_radii_m},
        'avg_depth': []
    }
    
    print(f"Processing: {input_path}")
    print(f"Resolution: {frame_width}x{frame_height}, FPS: {fps:.1f}")
    print(f"Testing radii: {ring_radii_m} meters")
    print("-" * 50)
    
    frame_count = 0
    
    # Process all frames
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # Adjust K if needed
        if frame_width != frame.shape[1] or frame_height != frame.shape[0]:
            frame_width, frame_height = frame.shape[1], frame.shape[0]
            K = get_default_K(frame_width, frame_height)
        
        # Run algorithm
        result = find_nearest_hoop_pose(frame, K, ring_radii_m)
        
        if len(result) == 2:
            result_data, intermediate_frames = result
            result_data = None
        else:
            center_3d, normal_3d, ellipse, used_radius, intermediate_frames = result
            result_data = (center_3d, normal_3d, ellipse, used_radius)
        
        # Initialize intermediate writers on the first frame if needed
        if frame_count == 0:
            for name in intermediate_file_names:
                debug_frame = intermediate_frames.get(name)
                if debug_frame is not None:
                    debug_path = os.path.join(output_dir, f"{name}.mp4")
                    
                    if debug_frame.ndim == 2:
                        debug_frame = cv2.cvtColor(debug_frame, cv2.COLOR_GRAY2BGR)
                    if debug_frame.shape[:2] != (frame_height, frame_width):
                        debug_frame = cv2.resize(debug_frame, (frame_width, frame_height))
                    
                    writer = cv2.VideoWriter(debug_path, fourcc, fps, (frame_width, frame_height))
                    intermediate_writers[name] = writer
        
        # Write intermediate frames
        for name in intermediate_file_names:
            writer = intermediate_writers.get(name)
            debug_frame = intermediate_frames.get(name)
            if writer is not None and debug_frame is not None:
                if debug_frame.ndim == 2:
                    debug_frame = cv2.cvtColor(debug_frame, cv2.COLOR_GRAY2BGR)
                # Resize only if necessary, though it should match or be 3-channel already
                if debug_frame.shape[:2] != (frame_height, frame_width):
                    debug_frame = cv2.resize(debug_frame, (frame_width, frame_height))
                
                cv2.putText(debug_frame, f"Frame {frame_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                writer.write(debug_frame)
        
        # --- Process Main Output ---
        frame_overlay = frame.copy()
        
        if result_data:
            center_3d, normal_3d, ellipse, used_radius = result_data
            # draw_hoop_pose_overlay internally uses the normal data
            draw_hoop_pose_overlay(frame_overlay, K, center_3d, normal_3d, ellipse, used_radius)
            
            info_text = [
                f"Frame: {frame_count}",
                f"Depth: {center_3d[2]:.2f}m",
                f"Radius: {used_radius:.3f}m",
                f"Normal: X:{normal_3d[0]:.2f}, Y:{normal_3d[1]:.2f}, Z:{normal_3d[2]:.2f}"
            ]
            
            for i, text in enumerate(info_text):
                cv2.putText(frame_overlay, text, (10, 30 + i*25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            stats['detected_frames'] += 1
            stats['radius_usage'][used_radius] += 1
            stats['avg_depth'].append(center_3d[2])
        else:
            cv2.putText(frame_overlay, f"Frame {frame_count}: No detection",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        out_final.write(frame_overlay)
        
        frame_count += 1
        stats['total_frames'] += 1
        
        # Progress
        if frame_count % 30 == 0:
            rate = (stats['detected_frames'] / frame_count) * 100
            print(f"Frame {frame_count}: Detection rate = {rate:.1f}%")
    
    # Cleanup
    cap.release()
    out_final.release()
    for writer in intermediate_writers.values():
        writer.release()
    
    # Calculate statistics (for console only)
    if stats['avg_depth']:
        stats['avg_depth'] = sum(stats['avg_depth']) / len(stats['avg_depth'])
    else:
        stats['avg_depth'] = 0
    
    # Print summary
    print("\n" + "="*50)
    print("Complete!")
    print(f"Output: {output_dir}")
    print(f"Frames: {stats['total_frames']}")
    print(f"Detected: {stats['detected_frames']} ({stats['detected_frames']/max(1,stats['total_frames'])*100:.1f}%)")
    if stats['avg_depth'] > 0:
        print(f"Avg depth: {stats['avg_depth']:.2f}m")
    print("\nRadius usage:")
    for r, count in stats['radius_usage'].items():
        if count > 0:
            pct = count/max(1,stats['detected_frames'])*100
            print(f"  {r:.3f}m: {count} frames ({pct:.1f}%)")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Hoop detection with debugging.")
    parser.add_argument('input_video', type=str, help='Input video path')
    parser.add_argument('--radii', type=float, nargs='+',
                       default=[0.2],
                       help='Ring radii to test (meters)')
    parser.add_argument('--base_output_dir', type=str,
                       default='SimpleAlgTestingOnRealSim',
                       help='Output directory')
    
    args = parser.parse_args()
    
    # Convert radii to float keys for dictionary use later
    radii = [float(r) for r in args.radii]
    
    run_dir = get_next_run_dir(args.base_output_dir)
    # The default intrinsic matrix K is calculated here based on the default W, H, and HFOV.
    K_matrix = get_default_K() 
    
    process_video_and_save_debug(args.input_video, run_dir, K_matrix, radii)