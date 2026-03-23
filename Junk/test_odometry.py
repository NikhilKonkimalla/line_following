import random
import time
from odometry import odometry

def generate_random_power_pairs(num_pairs=3):
    """
    Generate random power pairs for odometry testing.
    Power values are between 0.1 and 1.0 to ensure motors actually move.
    """
    power_pairs = []
    for _ in range(num_pairs):
        left_power = round(random.uniform(0.1, 1.0), 2)
        right_power = round(random.uniform(0.1, 1.0), 2)
        power_pairs.append([left_power, right_power])
    return power_pairs

def run_test(test_number, power_pairs):
    """
    Run a single odometry test with the given power pairs.
    """
    print(f"\n{'='*60}")
    print(f"TEST #{test_number}")
    print(f"{'='*60}")
    print(f"Power pairs: {power_pairs}")
    print(f"{'='*60}\n")
    
    result = odometry(power_pairs)
    
    print(f"\n{'='*60}")
    print(f"TEST #{test_number} RESULTS:")
    print(f"Power pairs: {power_pairs}")
    print(f"Final position: x={result[0]:.2f} inches, y={result[1]:.2f} inches")
    print(f"Final heading: {result[2]:.2f} radians ({result[2] * 180 / 3.14159:.1f}°)")
    print(f"{'='*60}\n")
    
    return result

def main():
    """
    Run multiple automated odometry tests with random power pairs.
    """
    num_tests = int(input("How many tests to run? "))
    num_pairs = int(input("How many power pairs per test? "))
    
    print(f"\nRunning {num_tests} tests with {num_pairs} power pairs each...")
    
    results = []
    for i in range(num_tests):
        power_pairs = generate_random_power_pairs(num_pairs)
        result = run_test(i + 1, power_pairs)
        results.append({
            'test_num': i + 1,
            'power_pairs': power_pairs,
            'x': result[0],
            'y': result[1],
            'theta': result[2]
        })
        
        # Wait a bit between tests to let motors cool down
        if i < num_tests - 1:
            print("Waiting 5 seconds before next test...")
            time.sleep(5)
    
    # Print summary
    print(f"\n{'='*60}")
    print(f"SUMMARY OF ALL {num_tests} TESTS")
    print(f"{'='*60}")
    for r in results:
        print(f"Test {r['test_num']}: {r['power_pairs']}")
        print(f"  → x={r['x']:.2f}, y={r['y']:.2f}, θ={r['theta']*180/3.14159:.1f}°")
    print(f"{'='*60}\n")

if __name__ == "__main__":
    main()
