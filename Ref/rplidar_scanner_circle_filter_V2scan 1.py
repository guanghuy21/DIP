from rplidar import RPLidar, RPLidarException
import time
import csv
import os
from datetime import datetime
import math
import statistics
import numpy as np

# Configuration
PORT = 'COM4'  # Change to your port
BAUDRATE = 115200
TIMEOUT = 3
MOTOR_STARTUP_DELAY = 3  # seconds to wait for motor stabilization
DEFAULT_SCAN_DURATION = 30  # Default scan duration in seconds
MAX_POINTS_TO_KEEP = 10000  # Maximum number of points to keep in final output

# Geometry calculation parameters
SAFETY_MARGIN_DEGREES = 5  # Extra degrees on each side to ensure full coverage

# Pre-scan settings for front-facing detection
PRESCAN_NUM_SCANS = 5  # Number of scans to collect for averaging
PRESCAN_MIN_QUALITY = 10  # Minimum quality for valid distance readings
PRESCAN_FRONT_ANGLE_TOLERANCE = 10  # +/- degrees around 0° (front of sensor) to look for object
# For RPLidar, 0° is typically the front. Adjust if your sensor orientation differs.

# Circle filtering parameters
CIRCLE_FIT_TOLERANCE_MM = 50  # Maximum deviation from circle in mm (adjust based on your needs)
MIN_POINTS_FOR_CIRCLE_FIT = 10  # Minimum points needed before applying circle filter
CIRCLE_UPDATE_INTERVAL = 5  # Update circle estimate every N scans

def polar_to_cartesian(angle_deg, distance_mm):
    """Convert polar coordinates to Cartesian (x, y) in mm."""
    angle_rad = math.radians(angle_deg)
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return x, y

def fit_circle(points):
    """
    Fit a circle to a set of (x, y) points using least squares.
    
    Args:
        points: List of (x, y) tuples in mm
        
    Returns:
        (center_x, center_y, radius) or None if fitting fails
    """
    if len(points) < 3:
        return None
    
    try:
        # Convert to numpy arrays
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        
        # Calculate coordinates of the barycenter
        x_m = np.mean(x)
        y_m = np.mean(y)
        
        # Calculate the reduced coordinates
        u = x - x_m
        v = y - y_m
        
        # Linear system for circle fitting
        Suv = np.sum(u * v)
        Suu = np.sum(u ** 2)
        Svv = np.sum(v ** 2)
        Suuv = np.sum(u ** 2 * v)
        Suvv = np.sum(u * v ** 2)
        Suuu = np.sum(u ** 3)
        Svvv = np.sum(v ** 3)
        
        # Solving the linear system
        A = np.array([[Suu, Suv], [Suv, Svv]])
        B = np.array([0.5 * (Suuu + Suvv), 0.5 * (Svvv + Suuv)])
        
        if np.linalg.det(A) == 0:
            return None
            
        uc, vc = np.linalg.solve(A, B)
        
        # Calculate circle parameters
        center_x = uc + x_m
        center_y = vc + y_m
        radius = np.sqrt(uc**2 + vc**2 + (Suu + Svv) / len(x))
        
        return center_x, center_y, radius
    except:
        return None

def distance_to_circle(point, circle_params):
    """
    Calculate the perpendicular distance from a point to a circle.
    
    Args:
        point: (x, y) tuple in mm
        circle_params: (center_x, center_y, radius) tuple in mm
        
    Returns:
        Distance in mm (positive if outside circle, negative if inside)
    """
    if circle_params is None:
        return 0
    
    center_x, center_y, radius = circle_params
    x, y = point
    
    # Distance from point to center
    dist_to_center = math.sqrt((x - center_x)**2 + (y - center_y)**2)
    
    # Distance to circle perimeter
    deviation = abs(dist_to_center - radius)
    
    return deviation

def filter_by_circle(points, expected_radius_mm, tolerance_mm=CIRCLE_FIT_TOLERANCE_MM):
    """
    Filter points based on how well they fit a circle.
    
    Args:
        points: List of (quality, angle, distance) tuples
        expected_radius_mm: Expected radius of the object in mm
        tolerance_mm: Maximum allowed deviation from circle in mm
        
    Returns:
        filtered_points: List of (quality, angle, distance) tuples that fit the circle
        circle_params: Fitted circle parameters (center_x, center_y, radius)
    """
    if len(points) < MIN_POINTS_FOR_CIRCLE_FIT:
        # Not enough points yet, return all points
        return points, None
    
    # Convert polar to Cartesian
    cartesian_points = [polar_to_cartesian(angle, distance) 
                       for quality, angle, distance in points]
    
    # Fit circle to points
    circle_params = fit_circle(cartesian_points)
    
    if circle_params is None:
        # Circle fitting failed, return all points
        return points, None
    
    center_x, center_y, fitted_radius = circle_params
    
    # Filter points based on deviation from fitted circle
    filtered = []
    for i, (quality, angle, distance) in enumerate(points):
        x, y = cartesian_points[i]
        deviation = distance_to_circle((x, y), circle_params)
        
        if deviation <= tolerance_mm:
            filtered.append((quality, angle, distance))
    
    return filtered, circle_params

def get_unique_filename(base_filename):
    """Generate a unique filename to avoid overwriting existing files."""
    if not os.path.exists(base_filename):
        return base_filename
    
    print(f"\nWARNING: File '{base_filename}' already exists!")
    print("Choose an option:")
    print("  1. Overwrite existing file")
    print("  2. Create new file with timestamp")
    print("  3. Create new file with counter (data_scan_1.csv, etc.)")
    print("  4. Enter custom filename")
    print("  5. Cancel scan")
    
    while True:
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == '1':
            confirm = input(f"Are you sure you want to overwrite '{base_filename}'? (y/n): ").strip().lower()
            if confirm == 'y':
                return base_filename
            else:
                print("Cancelled. Choose another option:")
                continue
                
        elif choice == '2':
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name, ext = os.path.splitext(base_filename)
            new_filename = f"{name}_{timestamp}{ext}"
            print(f"New filename: {new_filename}")
            return new_filename
            
        elif choice == '3':
            name, ext = os.path.splitext(base_filename)
            counter = 1
            while True:
                new_filename = f"{name}_{counter}{ext}"
                if not os.path.exists(new_filename):
                    print(f"New filename: {new_filename}")
                    return new_filename
                counter += 1
                
        elif choice == '4':
            custom = input("Enter new filename (e.g., 'my_scan.csv'): ").strip()
            if not custom.endswith('.csv'):
                custom += '.csv'
            if os.path.exists(custom):
                print(f"File '{custom}' already exists!")
                continue
            return custom
            
        elif choice == '5':
            print("Scan cancelled.")
            exit(0)
            
        else:
            print("Invalid choice. Please enter 1-5.")

def calculate_scan_angle(radius_cm, distance_cm, safety_margin=SAFETY_MARGIN_DEGREES):
    """Calculate the optimal scanning angle based on object geometry."""
    half_angle_rad = math.atan(radius_cm / distance_cm)
    half_angle_deg = math.degrees(half_angle_rad)
    half_angle_with_margin = half_angle_deg + safety_margin
    
    center_angle = 0
    min_angle = center_angle - half_angle_with_margin
    max_angle = center_angle + half_angle_with_margin
    
    if min_angle < 0:
        min_angle += 360
    if max_angle < 0:
        max_angle += 360
    
    total_angle = half_angle_with_margin * 2
    
    return min_angle, max_angle, half_angle_deg, total_angle

def auto_detect_distance(lidar, num_scans=PRESCAN_NUM_SCANS, 
                         min_quality=PRESCAN_MIN_QUALITY,
                         angle_tolerance=PRESCAN_FRONT_ANGLE_TOLERANCE):
    """
    Perform a front-facing pre-scan to automatically detect object distance.
    Only looks for objects directly in front of the sensor (around 0°).
    
    Args:
        lidar: RPLidar object
        num_scans: Number of scans to collect for averaging
        min_quality: Minimum quality threshold for valid readings
        angle_tolerance: +/- degrees around 0° to search for object
        
    Returns:
        detected_distance: Average distance to object in front (mm)
        detected_angle: Average angle where object was found (should be ~0°)
    """
    print("\n" + "="*60)
    print("AUTO-DETECTION: FRONT-FACING SCAN")
    print("="*60)
    print(f"Performing {num_scans} scans to detect object in front...")
    print(f"Search range: {360-angle_tolerance:.1f}°-360° and 0°-{angle_tolerance:.1f}°")
    print(f"Quality threshold: >= {min_quality}")
    print(f"NOTE: 0° is considered the front of the sensor")
    print()
    
    # Helper function to check if angle is within front-facing range
    def is_front_facing(angle):
        """Check if angle is within tolerance of 0° (front)."""
        # Handle angles near 0° (can be 359-360 or 0-1)
        if angle <= angle_tolerance:
            return True
        if angle >= (360 - angle_tolerance):
            return True
        return False
    
    # Collect distance readings from front-facing region
    front_distances = []
    front_angles = []
    scan_count = 0
    
    for scan in lidar.iter_scans(max_buf_meas=5000):
        scan_count += 1
        
        # Collect points in front-facing region with good quality
        valid_points_this_scan = 0
        for quality, angle, distance in scan:
            if quality >= min_quality and is_front_facing(angle):
                front_distances.append(distance)
                front_angles.append(angle)
                valid_points_this_scan += 1
        
        print(f"  Scan {scan_count}/{num_scans}: Found {valid_points_this_scan} valid points in front")
        
        if scan_count >= num_scans:
            break
    
    print()
    
    # Analyze collected distances
    if not front_distances:
        print("❌ No valid objects detected in front!")
        print("\nPossible reasons:")
        print("  - No objects directly in front of sensor")
        print("  - Object is outside the front-facing detection range")
        print("  - Object is transparent/reflective")
        print("  - Quality threshold too high")
        print(f"\nTIP: Increase angle tolerance (currently ±{angle_tolerance}°)")
        print("     or manually enter the distance.")
        return None, None
    
    # Calculate average distance and angle
    avg_distance = statistics.mean(front_distances)
    median_distance = statistics.median(front_distances)
    min_distance = min(front_distances)
    max_distance = max(front_distances)
    std_dev = statistics.stdev(front_distances) if len(front_distances) > 1 else 0
    
    # Normalize angles near 0° (convert 359° to -1° for averaging)
    normalized_angles = []
    for angle in front_angles:
        if angle > 180:
            normalized_angles.append(angle - 360)
        else:
            normalized_angles.append(angle)
    
    avg_angle = statistics.mean(normalized_angles)
    # Convert back to 0-360 range
    if avg_angle < 0:
        avg_angle += 360
    
    print(f"✅ Object detected in front!")
    print(f"\nDistance Statistics ({len(front_distances)} readings):")
    print(f"  Average:  {avg_distance:.2f} mm ({avg_distance/10:.2f} cm) ⭐")
    print(f"  Median:   {median_distance:.2f} mm ({median_distance/10:.2f} cm)")
    print(f"  Minimum:  {min_distance:.2f} mm ({min_distance/10:.2f} cm)")
    print(f"  Maximum:  {max_distance:.2f} mm ({max_distance/10:.2f} cm)")
    print(f"  Std Dev:  {std_dev:.2f} mm")
    print(f"\nDetected object:")
    print(f"  Distance: {avg_distance:.2f} mm ({avg_distance/10:.2f} cm)")
    print(f"  Direction: {avg_angle:.1f}° (should be ~0° for centered object)")
    
    if abs(avg_angle) > 5 and abs(avg_angle - 360) > 5:
        print(f"\n⚠️  WARNING: Object not perfectly centered (angle: {avg_angle:.1f}°)")
        print(f"    Consider adjusting sensor alignment for best results")
    
    return avg_distance, avg_angle

def get_user_input_with_autodetect():
    """Get scanning parameters from user with auto-detection option."""
    print("="*60)
    print("RPLIDAR SCANNER WITH CIRCLE FILTERING")
    print("="*60)
    print()
    
    # Get object radius
    while True:
        try:
            radius_input = input("Enter object radius in cm: ").strip()
            radius = float(radius_input)
            if radius > 0:
                break
            print("Radius must be positive!")
        except ValueError:
            print("Please enter a valid number!")
    
    # Ask if user wants auto-detection
    auto_detect = input("\nUse auto-detection to find object distance? (y/n): ").strip().lower() == 'y'
    
    if not auto_detect:
        # Get distance manually
        while True:
            try:
                distance_input = input("Enter distance to object center in cm: ").strip()
                distance = float(distance_input)
                if distance > 0 and distance > radius:
                    break
                print("Distance must be positive and greater than radius!")
            except ValueError:
                print("Please enter a valid number!")
    else:
        distance = None  # Will be auto-detected
    
    # Get safety margin
    margin_input = input(f"\nEnter safety margin in degrees (default {SAFETY_MARGIN_DEGREES}): ").strip()
    margin = float(margin_input) if margin_input else SAFETY_MARGIN_DEGREES
    
    # Get scan duration
    duration_input = input(f"Enter scan duration in seconds (default {DEFAULT_SCAN_DURATION}): ").strip()
    scan_duration = float(duration_input) if duration_input else DEFAULT_SCAN_DURATION
    
    # Get circle filtering parameters
    print("\n" + "="*60)
    print("CIRCLE FILTERING SETTINGS")
    print("="*60)
    tolerance_input = input(f"Enter circle fit tolerance in mm (default {CIRCLE_FIT_TOLERANCE_MM}): ").strip()
    tolerance = float(tolerance_input) if tolerance_input else CIRCLE_FIT_TOLERANCE_MM
    print(f"Points deviating more than {tolerance} mm from fitted circle will be rejected.")
    
    return radius, distance, margin, scan_duration, auto_detect, tolerance

def initialize_csv(filename, radius, distance, min_angle, max_angle, duration, auto_detected, tolerance):
    """Initialize CSV file with header and metadata."""
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['# RPLIDAR Scan Data with Circle Filtering'])
        writer.writerow([f'# Timestamp: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}'])
        writer.writerow([f'# Object Radius: {radius:.2f} cm'])
        writer.writerow([f'# Sensor Distance: {distance:.2f} cm {"(Auto-detected)" if auto_detected else "(Manual)"}'])
        writer.writerow([f'# Scan Range: {min_angle:.2f}° to {max_angle:.2f}°'])
        writer.writerow([f'# Scan Duration: {duration:.1f} seconds'])
        writer.writerow([f'# Circle Fit Tolerance: {tolerance:.1f} mm'])
        writer.writerow([f'# Max Points: {MAX_POINTS_TO_KEEP}'])
        writer.writerow([''])
        writer.writerow(['angle', 'distance', 'quality'])

def write_final_data_to_csv(filename, data_buffer):
    """Append all buffered data to CSV file."""
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for quality, angle, distance in data_buffer:
            writer.writerow([f'{angle:.2f}', f'{distance:.2f}', quality])

def filter_scan_data(scan, min_angle, max_angle):
    """Filter scan data to only include points within the specified angle range."""
    filtered = []
    
    # Handle wrap-around case (e.g., 350° to 10°)
    wraps_around = min_angle > max_angle
    
    for quality, angle, distance in scan:
        if wraps_around:
            # Check if angle is either >= min_angle OR <= max_angle
            if angle >= min_angle or angle <= max_angle:
                filtered.append((quality, angle, distance))
        else:
            # Normal case: check if min_angle <= angle <= max_angle
            if min_angle <= angle <= max_angle:
                filtered.append((quality, angle, distance))
    
    return filtered

def print_scan_statistics(scan_count, filtered_count, total_count, buffer_size, elapsed_time, total_time):
    """Print scan statistics with progress."""
    progress = (elapsed_time / total_time) * 100
    print(f"\rScan #{scan_count:4d} | "
          f"Filtered: {filtered_count:4d}/{total_count:4d} pts | "
          f"Buffer: {buffer_size:5d}/{MAX_POINTS_TO_KEEP} | "
          f"Progress: {progress:5.1f}% ({elapsed_time:.1f}/{total_time:.1f}s)", 
          end='', flush=True)

def print_sample_points(filtered_scan, num_samples=3):
    """Print sample points from filtered scan."""
    samples = filtered_scan[:num_samples]
    print(f"\n  Sample points (first {len(samples)}):")
    for quality, angle, distance in samples:
        print(f"    Q:{quality:3d} | Angle:{angle:6.2f}° | Dist:{distance:7.2f}mm ({distance/10:.2f}cm)")

def cleanup_lidar(lidar):
    """Clean up LIDAR connection."""
    try:
        print("\n\nStopping LIDAR...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("LIDAR stopped and disconnected.")
    except:
        pass

def main():
    """Main function."""
    lidar = None
    
    try:
        # Get user input
        radius, distance, margin, scan_duration, auto_detect, circle_tolerance = get_user_input_with_autodetect()
        
        # Connect to LIDAR
        print(f"\nConnecting to RPLIDAR on {PORT}...")
        lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        
        # Get device info
        info = lidar.get_info()
        print(f"✅ Connected to RPLIDAR")
        print(f"   Model: {info['model']}")
        print(f"   Firmware: {info['firmware'][0]}.{info['firmware'][1]}")
        print(f"   Hardware: {info['hardware']}")
        print(f"   Serial: {info['serialnumber']}")
        
        # Start motor and wait for stabilization
        print(f"\n🔄 Starting motor and waiting {MOTOR_STARTUP_DELAY} seconds for stabilization...")
        lidar.start_motor()
        time.sleep(MOTOR_STARTUP_DELAY)
        
        # Auto-detection if requested
        auto_detected = False
        if auto_detect:
            detected_distance_mm, detected_angle = auto_detect_distance(lidar)
            
            if detected_distance_mm is None:
                print("\n⚠️  Auto-detection failed!")
                
                manual = input("\nEnter distance manually? (y/n): ").strip().lower()
                if manual == 'y':
                    # Manual override
                    while True:
                        try:
                            distance_input = input("Enter distance to YOUR target object in cm: ").strip()
                            distance = float(distance_input)
                            if distance > 0 and distance > radius:
                                break
                            print("Invalid distance!")
                        except ValueError:
                            print("Please enter a valid number!")
                    auto_detected = False
                else:
                    print("Please reposition sensor and run the program again.")
                    cleanup_lidar(lidar)
                    return
            else:
                auto_detected = True
                distance = detected_distance_mm / 10  # Convert to cm
                print(f"\n✅ Using detected distance: {distance:.2f} cm")
        else:
            auto_detected = False
        
        # Calculate angles
        min_angle, max_angle, half_angle, total_angle = calculate_scan_angle(radius, distance, margin)
        expected_radius_mm = radius * 10  # Convert to mm
        
        # Display calculation results
        print("\n" + "="*60)
        print("CALCULATION RESULTS")
        print("="*60)
        print(f"Object radius:           {radius:.2f} cm ({expected_radius_mm:.2f} mm)")
        print(f"Sensor distance:         {distance:.2f} cm {'(AUTO-DETECTED)' if auto_detected else ''}")
        print(f"Safety margin:           {margin:.2f}°")
        print(f"Scan duration:           {scan_duration:.1f} seconds")
        print(f"Circle fit tolerance:    {circle_tolerance:.2f} mm")
        print(f"\nGeometric half-angle:    {half_angle:.2f}°")
        print(f"Total scanning angle:    {total_angle:.2f}°")
        print(f"\nScan range:              {min_angle:.2f}° to {max_angle:.2f}°")
        print(f"Angular coverage:        {max_angle - min_angle:.2f}°")
        print("="*60)
        
        # Ask for confirmation
        confirm = input("\nProceed with focused scan? (y/n): ").strip().lower()
        
        if confirm != 'y':
            print("Scan cancelled by user.")
            cleanup_lidar(lidar)
            return
        
        # Get output filename
        default_output = "data_scan_circle_filtered.csv"
        output_file = get_unique_filename(default_output)
        
        print(f"\nOutput file: {output_file}")
        
        # Need to restart scan after pre-scan
        if auto_detect:
            print("\nRestarting for focused scan...")
            lidar.stop()
            time.sleep(0.5)
        
        # Initialize CSV
        print(f"\nStarting focused scan with circle filtering...")
        print(f"Angle filter: {min_angle:.2f}° to {max_angle:.2f}°")
        print(f"Circle filter: ±{circle_tolerance:.2f} mm from fitted circle")
        initialize_csv(output_file, radius, distance, min_angle, max_angle, scan_duration, auto_detected, circle_tolerance)
        
        # Scan variables
        data_buffer = []  # Buffer to store best-fitting points
        total_points_collected = 0  # Total points seen (after angle filter)
        points_rejected_angle = 0  # Points rejected by angle filter
        points_rejected_circle = 0  # Points rejected by circle filter
        points_rejected_buffer = 0  # Points rejected due to buffer being full
        scan_count = 0
        start_time = time.time()
        scan_end_time = start_time + scan_duration
        
        # Circle fitting variables
        circle_params = None
        last_circle_update = 0
        
        print(f"\nScan started at: {datetime.now().strftime('%H:%M:%S')}")
        print(f"Scan will end at: {datetime.fromtimestamp(scan_end_time).strftime('%H:%M:%S')}")
        print(f"Buffer limit: {MAX_POINTS_TO_KEEP} points")
        print(f"Strategy: Two-layer filtering (angle + circle fit)")
        print()
        
        # Main scanning loop - run for specified duration
        for scan in lidar.iter_scans(max_buf_meas=50000):
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Check if scan duration has been reached
            if elapsed_time >= scan_duration:
                print(f"\n\n⏱️  Scan duration of {scan_duration:.1f} seconds reached. Stopping...")
                break
            
            # LAYER 1: Filter by angle
            points_before_angle_filter = len(scan)
            filtered_by_angle = filter_scan_data(scan, min_angle, max_angle)
            points_after_angle_filter = len(filtered_by_angle)
            points_rejected_angle += (points_before_angle_filter - points_after_angle_filter)
            
            # Update circle fit periodically
            if scan_count - last_circle_update >= CIRCLE_UPDATE_INTERVAL and len(data_buffer) >= MIN_POINTS_FOR_CIRCLE_FIT:
                _, circle_params = filter_by_circle(data_buffer, expected_radius_mm, circle_tolerance)
                last_circle_update = scan_count
                
                if circle_params and scan_count == CIRCLE_UPDATE_INTERVAL:
                    cx, cy, r = circle_params
                    print(f"\n📊 Circle fitted: center=({cx:.1f}, {cy:.1f}) mm, radius={r:.1f} mm (expected: {expected_radius_mm:.1f} mm)")
            
            # LAYER 2: Filter by circle (if we have enough points)
            if circle_params is not None:
                filtered_by_circle, _ = filter_by_circle(filtered_by_angle, expected_radius_mm, circle_tolerance)
                points_rejected_circle += (points_after_angle_filter - len(filtered_by_circle))
                filtered_scan = filtered_by_circle
            else:
                filtered_scan = filtered_by_angle
            
            # Add filtered points to buffer with quality-based selection
            for quality, angle, distance in filtered_scan:
                total_points_collected += 1
                
                if len(data_buffer) < MAX_POINTS_TO_KEEP:
                    # Buffer not full yet, add point directly
                    data_buffer.append((quality, angle, distance))
                else:
                    # Buffer is full - compare with worst point in buffer
                    worst_quality = min(data_buffer, key=lambda p: p[0])[0]
            
                    # If new point has better quality, replace worst point
                    if quality > worst_quality:
                        # Remove the worst quality point
                        worst_point = min(data_buffer, key=lambda p: p[0])
                        data_buffer.remove(worst_point)
                        # Add the better quality point
                        data_buffer.append((quality, angle, distance))
                        points_rejected_buffer += 1
                    else:
                        # New point is not better, reject it
                        points_rejected_buffer += 1
            
            # Print statistics
            circle_status = "✓" if circle_params else "✗"
            print(f"\rScan #{scan_count:4d} | "
                  f"Angle: {points_after_angle_filter:4d}/{points_before_angle_filter:4d} | "
                  f"Circle {circle_status}: {len(filtered_scan):4d} | "
                  f"Buffer: {len(data_buffer):5d}/{MAX_POINTS_TO_KEEP} | "
                  f"Progress: {(elapsed_time/scan_duration)*100:5.1f}%", 
                  end='', flush=True)
            
            # Show sample points for first few scans
            if scan_count < 3 and len(filtered_scan) > 0:
                print_sample_points(filtered_scan)
            
            scan_count += 1
        
        # Sort buffer by angle for cleaner output
        data_buffer.sort(key=lambda p: p[1])  # Sort by angle
        
        # Final circle fit on all saved points
        print(f"\n\n🔄 Performing final circle fit on {len(data_buffer)} saved points...")
        _, final_circle = filter_by_circle(data_buffer, expected_radius_mm, circle_tolerance)
        
        # Write buffered data to CSV at the end
        print(f"💾 Writing {len(data_buffer)} best-fitting points to CSV...")
        write_final_data_to_csv(output_file, data_buffer)
        
        # Print completion statistics
        actual_scan_time = time.time() - start_time
        
        # Calculate quality statistics of saved points
        if data_buffer:
            qualities = [q for q, a, d in data_buffer]
            distances = [d for q, a, d in data_buffer]
            avg_quality = sum(qualities) / len(qualities)
            min_quality = min(qualities)
            max_quality = max(qualities)
            avg_distance = sum(distances) / len(distances)
            min_distance = min(distances)
            max_distance = max(distances)
        else:
            avg_quality = min_quality = max_quality = 0
            avg_distance = min_distance = max_distance = 0
        
        print(f"\n{'='*60}")
        print(f"SCAN COMPLETE!")
        print(f"{'='*60}")
        print(f"Object radius:           {radius:.2f} cm ({expected_radius_mm:.2f} mm)")
        print(f"Sensor distance:         {distance:.2f} cm {'(AUTO-DETECTED)' if auto_detected else ''}")
        print(f"Scan range:              {min_angle:.2f}° to {max_angle:.2f}°")
        print(f"Circle tolerance:        ±{circle_tolerance:.2f} mm")
        
        print(f"\nTIMING:")
        print(f"  Requested duration:    {scan_duration:.1f} seconds")
        print(f"  Actual scan time:      {actual_scan_time:.1f} seconds")
        print(f"  Total scans:           {scan_count}")
        print(f"  Average scan rate:     {scan_count/actual_scan_time:.1f} scans/sec")
        
        total_points_seen = total_points_collected + points_rejected_angle + points_rejected_circle + points_rejected_buffer
        print(f"\nFILTERING RESULTS:")
        print(f"  Total points seen:     {total_points_seen}")
        print(f"  Rejected by angle:     {points_rejected_angle} ({points_rejected_angle/max(total_points_seen, 1)*100:.1f}%)")
        print(f"  Rejected by circle:    {points_rejected_circle} ({points_rejected_circle/max(total_points_seen, 1)*100:.1f}%)")
        print(f"  Rejected by buffer:    {points_rejected_buffer} ({points_rejected_buffer/max(total_points_seen, 1)*100:.1f}%)")
        print(f"  Points saved:          {len(data_buffer)} ({len(data_buffer)/max(total_points_seen, 1)*100:.1f}%)")
        
        print(f"\nQUALITY OF SAVED POINTS:")
        print(f"  Average quality:       {avg_quality:.1f}")
        print(f"  Min quality:           {min_quality}")
        print(f"  Max quality:           {max_quality}")
        
        print(f"\nDISTANCE STATISTICS:")
        print(f"  Average distance:      {avg_distance:.2f} mm ({avg_distance/10:.2f} cm)")
        print(f"  Min distance:          {min_distance:.2f} mm ({min_distance/10:.2f} cm)")
        print(f"  Max distance:          {max_distance:.2f} mm ({max_distance/10:.2f} cm)")
        
        if final_circle:
            cx, cy, fitted_r = final_circle
            radius_error = abs(fitted_r - expected_radius_mm)
            print(f"\nFINAL CIRCLE FIT:")
            print(f"  Center:                ({cx:.2f}, {cy:.2f}) mm")
            print(f"  Fitted radius:         {fitted_r:.2f} mm ({fitted_r/10:.2f} cm)")
            print(f"  Expected radius:       {expected_radius_mm:.2f} mm ({radius:.2f} cm)")
            print(f"  Radius error:          {radius_error:.2f} mm ({radius_error/10:.2f} cm)")
            print(f"  Error percentage:      {radius_error/expected_radius_mm*100:.2f}%")
        
        print(f"\nData saved to:           {os.path.abspath(output_file)}")
        print(f"{'='*60}")
        
    except RPLidarException as e:
        print(f"\nRPLIDAR error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Try unplugging and replugging the USB cable")
        print("2. Make sure the motor is spinning (you should hear it)")
        print("3. Try a different USB port (USB 2.0 preferred)")
        print("4. Check if another program is using the LIDAR")
        print("5. Verify the COM port is correct in Device Manager (Windows)")
        
    except KeyboardInterrupt:
        print("\n\nScan interrupted by user (Ctrl+C)")
        if 'output_file' in locals():
            print(f"Partial data saved to: {output_file}")
        
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        if lidar:
            cleanup_lidar(lidar)

if __name__ == "__main__":
    main()