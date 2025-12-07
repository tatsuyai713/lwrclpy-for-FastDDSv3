#!/usr/bin/env python3
"""
lwrclpy examples test script for Ubuntu

This script tests all lwrclpy example code to ensure it runs correctly on Ubuntu.
Tests are executed from /tmp directory to avoid source code interference.
"""

import subprocess
import sys
import signal
from pathlib import Path

# Store test results
test_results = []

# Get project root (parent of test directory)
PROJECT_ROOT = Path(__file__).parent.parent.resolve()


class Colors:
    """
    Terminal color codes for test result output
    """
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


def print_header(text):
    """Print section header"""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'=' * 70}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'=' * 70}{Colors.RESET}\n")


def print_test_start(test_name):
    """Print test start message"""
    print(f"\n{Colors.BOLD}Testing: {test_name}{Colors.RESET}")
    print("-" * 70)


def print_success(message):
    """Print success message"""
    print(f"{Colors.GREEN}✓ {message}{Colors.RESET}")


def print_error(message):
    """Print error message"""
    print(f"{Colors.RED}✗ {message}{Colors.RESET}")


def print_warning(message):
    """Print warning message"""
    print(f"{Colors.YELLOW}⚠ {message}{Colors.RESET}")


def run_example_with_timeout(script_path, timeout=15.0, check_output=None):
    """
    Execute example script with timeout
    
    Args:
        script_path: Path to the script to execute
        timeout: Timeout in seconds (default: 15.0)
        check_output: List of keywords to check in output (optional)
    
    Returns:
        Tuple of (success: bool, output: str)
    """
    try:
        # Execute from /tmp to avoid source code interference
        process = subprocess.Popen(
            ["python3", str(script_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd="/tmp",
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
        )
        
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate()
            return False, "Timeout"
        
        output = stdout + stderr
        
        # Check for errors
        if "Traceback" in output or "Error" in output:
            # Allow expected errors in listener examples
            if "listener.py" in str(script_path) and "timeout" in output.lower():
                return True, output
            return False, output
        
        # Check for specific output keywords if specified
        if check_output:
            for keyword in check_output:
                if keyword not in output:
                    return False, f"Expected keyword '{keyword}' not found in output"
        
        return True, output
        
    except Exception as e:
        return False, f"Error occurred: {str(e)}"


def test_basic_pubsub():
    """Test basic Pub/Sub"""
    print_test_start("Basic Publisher/Subscriber")
    
    examples = [
        (PROJECT_ROOT / "examples/pubsub/string/talker.py", None),
        (PROJECT_ROOT / "examples/pubsub/string/listener.py", None),
    ]
    
    for script, keywords in examples:
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords)
        relative_path = script.relative_to(PROJECT_ROOT)
        test_results.append((str(relative_path), success))
        
        if success:
            print_success(f"{relative_path} - OK")
        else:
            print_error(f"{relative_path} - FAILED")
            print(f"  {output}")


def test_timers():
    """Test timers"""
    print_test_start("Timers")
    
    examples = [
        (PROJECT_ROOT / "examples/timers/wall_timer.py", None),
        (PROJECT_ROOT / "examples/timers/wall_timer_listener.py", None),
        (PROJECT_ROOT / "examples/timers/oneshot_and_periodic.py", None),
        (PROJECT_ROOT / "examples/timers/oneshot_and_periodic_listener.py", None),
    ]
    
    for script, keywords in examples:
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords)
        relative_path = script.relative_to(PROJECT_ROOT)
        test_results.append((str(relative_path), success))
        
        if success:
            print_success(f"{relative_path} - OK")
        else:
            print_error(f"{relative_path} - FAILED")
            print(f"  {output}")


def test_parameters():
    """Test parameters"""
    print_test_start("Parameters")
    
    script = PROJECT_ROOT / "examples/parameters/logger_and_params.py"
    success, output = run_example_with_timeout(script, timeout=15.0, check_output=None)
    relative_path = script.relative_to(PROJECT_ROOT)
    test_results.append((str(relative_path), success))
    
    if success:
        print_success(f"{relative_path} - OK")
    else:
        print_error(f"{relative_path} - FAILED")
        print(f"  {output}")


def test_executor():
    """Test executors"""
    print_test_start("Executors")
    
    examples = [
        (PROJECT_ROOT / "examples/executor/single_node_spin.py", None),
        (PROJECT_ROOT / "examples/executor/multithreaded_spin.py", None),
    ]
    
    for script, keywords in examples:
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords)
        relative_path = script.relative_to(PROJECT_ROOT)
        test_results.append((str(relative_path), success))
        
        if success:
            print_success(f"{relative_path} - OK")
        else:
            print_error(f"{relative_path} - FAILED")
            print(f"  {output}")


def test_guard_condition():
    """Test guard condition"""
    print_test_start("Guard Condition")
    
    script = PROJECT_ROOT / "examples/guard_condition/trigger_guard_condition.py"
    success, output = run_example_with_timeout(script, timeout=15.0, check_output=None)
    relative_path = script.relative_to(PROJECT_ROOT)
    test_results.append((str(relative_path), success))
    
    if success:
        print_success(f"{relative_path} - OK")
    else:
        print_error(f"{relative_path} - FAILED")
        print(f"  {output}")


def test_services():
    """Test services"""
    print_test_start("Services")
    
    # Services are client/server pairs, so test individually
    examples = [
        (PROJECT_ROOT / "examples/services/set_bool/server.py", None),
        (PROJECT_ROOT / "examples/services/set_bool/client.py", None),
        (PROJECT_ROOT / "examples/services/trigger_bridge/bridge.py", None),
    ]
    
    for script, keywords in examples:
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords)
        relative_path = script.relative_to(PROJECT_ROOT)
        test_results.append((str(relative_path), success))
        
        if success:
            print_success(f"{relative_path} - OK")
        else:
            print_error(f"{relative_path} - FAILED")
            print(f"  {output}")


def test_actions():
    """Test actions"""
    print_test_start("Actions")
    
    examples = [
        (PROJECT_ROOT / "examples/actions/fibonacci_action_server.py", None),
        (PROJECT_ROOT / "examples/actions/fibonacci_action_client.py", None),
    ]
    
    for script, keywords in examples:
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords)
        relative_path = script.relative_to(PROJECT_ROOT)
        test_results.append((str(relative_path), success))
        
        if success:
            print_success(f"{relative_path} - OK")
        else:
            print_error(f"{relative_path} - FAILED")
            print(f"  {output}")


def test_import():
    """Basic import test"""
    print_test_start("Import Test")
    
    try:
        import lwrclpy
        from std_msgs.msg import String
        from sensor_msgs.msg import Image
        from geometry_msgs.msg import Pose
        
        print_success("lwrclpy and message types imported successfully")
        test_results.append(("import_test", True))
        return True
    except Exception as e:
        print_error(f"Import failed: {str(e)}")
        test_results.append(("import_test", False))
        return False


def test_simple_publish():
    """Simple publish test"""
    print_test_start("Simple Publish Test")
    
    # Execute in separate process to avoid hanging
    test_code = """
import sys
import lwrclpy as rclpy
from std_msgs.msg import String

print('Initializing...', flush=True)
rclpy.init()
print('Creating node...', flush=True)
node = rclpy.create_node('test_node')
print('Creating publisher...', flush=True)
pub = node.create_publisher(String, 'test_topic', 10)

msg = String()
msg.data = 'Test message from Ubuntu'
print('Publishing message...', flush=True)
pub.publish(msg)
print('Published message successfully', flush=True)

node.destroy_node()
rclpy.shutdown()
print('Shutdown complete', flush=True)
"""
    
    try:
        process = subprocess.Popen(
            [sys.executable, '-c', test_code],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd='/tmp'
        )
        
        try:
            stdout, stderr = process.communicate(timeout=15.0)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate()
            print_error("Simple publish test timed out")
            test_results.append(("simple_publish", False))
            return False
        
        output = stdout + stderr
        
        if "Published message successfully" in output and "Traceback" not in output:
            print_success("Simple publish test completed successfully")
            test_results.append(("simple_publish", True))
            return True
        else:
            print_error(f"Simple publish test failed: {output[:300]}")
            test_results.append(("simple_publish", False))
            return False
            
    except Exception as e:
        print_error(f"Simple publish test failed: {str(e)}")
        test_results.append(("simple_publish", False))
        return False


def print_summary():
    """Output test result summary"""
    print_header("Test Summary")
    
    passed = sum(1 for _, success in test_results if success)
    failed = sum(1 for _, success in test_results if not success)
    total = len(test_results)
    
    print(f"Total tests: {total}")
    print(f"{Colors.GREEN}Passed: {passed}{Colors.RESET}")
    print(f"{Colors.RED}Failed: {failed}{Colors.RESET}")
    print(f"Success rate: {passed/total*100:.1f}%\n")
    
    if failed > 0:
        print(f"{Colors.RED}Failed tests:{Colors.RESET}")
        for name, success in test_results:
            if not success:
                print(f"  - {name}")
        print()
    
    return failed == 0


def main():
    """Main function"""
    print_header("lwrclpy Examples Test Suite for Ubuntu")
    
    print(f"Python: {sys.version}")
    print(f"Project root: {PROJECT_ROOT}")
    print(f"Working directory: /tmp\n")
    
    # Basic import test
    if not test_import():
        print_error("Import test failed. Aborting further tests.")
        sys.exit(1)
    
    # Simple publish test
    test_simple_publish()
    
    # Test each category
    test_basic_pubsub()
    test_timers()
    test_parameters()
    test_executor()
    test_guard_condition()
    test_services()
    test_actions()
    
    # Output summary
    all_passed = print_summary()
    
    if all_passed:
        print_success("All tests passed! ✨")
        sys.exit(0)
    else:
        print_error("Some tests failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
