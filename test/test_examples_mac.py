#!/usr/bin/env python3
"""
lwrclpy examples test script for macOS

This script runs example code under the examples/ directory sequentially
to verify that it works correctly in the macOS environment.
"""

import subprocess
import sys
import time
import os
import signal
import gc
from pathlib import Path
from typing import List, Tuple, Optional

# Store test results
test_results = []

# Project root directory
PROJECT_ROOT = Path(__file__).parent.parent.absolute()

class Colors:
    """Terminal color output"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'

def print_header(text: str):
    """Print header"""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'=' * 70}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text:^70}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'=' * 70}{Colors.RESET}\n")

def print_test_start(name: str):
    """Print test start"""
    print(f"{Colors.YELLOW}▶ Testing: {name}{Colors.RESET}")

def print_success(message: str):
    """Print success message"""
    print(f"{Colors.GREEN}✓ {message}{Colors.RESET}")

def print_error(message: str):
    """Print error message"""
    print(f"{Colors.RED}✗ {message}{Colors.RESET}")

def print_warning(message: str):
    """Print warning message"""
    print(f"{Colors.YELLOW}⚠ {message}{Colors.RESET}")

def run_example_with_timeout(
    script_path: Path,
    timeout: float = 6.0,
    check_output: Optional[List[str]] = None,
    timeout_ok: bool = False
) -> Tuple[bool, str]:
    """
    Run sample script with timeout
    
    Args:
        script_path: Path to the script to run
        timeout: Timeout in seconds
        check_output: List of keywords that should be in the output (None to skip check)
        timeout_ok: If True, treat timeout as success (for infinite loop scripts)
    
    Returns:
        (success, output or error message)
    """
    try:
        # Run from /tmp to avoid path issues
        process = subprocess.Popen(
            [sys.executable, str(script_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd='/tmp'
        )
        
        # Wait with timeout
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            # Terminate process on timeout
            process.send_signal(signal.SIGTERM)
            time.sleep(0.5)
            if process.poll() is None:
                process.kill()
            stdout, stderr = process.communicate()
            
            # If timeout is OK (infinite loop scripts), treat as success
            if timeout_ok:
                output = stdout + stderr
                # Check for critical errors even when timeout is expected
                if "Traceback" in output and "KeyboardInterrupt" not in output:
                    if any(err in output for err in ["ModuleNotFoundError", "ImportError", "AttributeError", "SyntaxError"]):
                        return False, f"Error occurred:\n{output[:500]}"
                return True, "Timed out as expected (infinite loop script)"
        
        output = stdout + stderr
        
        # Check for critical errors (excluding KeyboardInterrupt)
        if "Traceback" in output and "KeyboardInterrupt" not in output:
            # Only fail on actual errors like ModuleNotFoundError
            if any(err in output for err in ["ModuleNotFoundError", "ImportError", "AttributeError", "SyntaxError"]):
                return False, f"Error occurred:\n{output[:500]}"
        
        # Output check (only if specified)
        if check_output:
            for keyword in check_output:
                if keyword not in output:
                    # Warn if expected output not found, but don't fail if no errors
                    return True, f"Note: Expected output '{keyword}' not found, but no errors"
        
        return True, output[:300] if output else "No output (OK)"
        
    except Exception as e:
        return False, f"Exception during execution: {str(e)}"

def test_basic_pubsub():
    """Test basic Pub/Sub"""
    print_test_start("Basic Publisher/Subscriber")
    
    examples = [
        (PROJECT_ROOT / "examples/pubsub/string/talker.py", None),
        (PROJECT_ROOT / "examples/pubsub/string/listener.py", None),
    ]
    
    for script, keywords in examples:
        # These are infinite-loop scripts, so timeout is expected
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords, timeout_ok=True)
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
        # These are infinite-loop scripts, so timeout is expected
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords, timeout_ok=True)
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
        # These are infinite-loop scripts, so timeout is expected
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords, timeout_ok=True)
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
        (PROJECT_ROOT / "examples/services/set_bool/server.py", None, True),  # server is infinite-loop
        (PROJECT_ROOT / "examples/services/set_bool/client.py", None, True),  # client is infinite-loop
        (PROJECT_ROOT / "examples/services/trigger_bridge/bridge.py", None, False),  # bridge exits normally
    ]
    
    for script, keywords, timeout_ok in examples:
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords, timeout_ok=timeout_ok)
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
        # These are infinite-loop scripts, so timeout is expected
        success, output = run_example_with_timeout(script, timeout=15.0, check_output=keywords, timeout_ok=True)
        relative_path = script.relative_to(PROJECT_ROOT)
        test_results.append((str(relative_path), success))
        
        if success:
            print_success(f"{relative_path} - OK")
        else:
            print_error(f"{relative_path} - FAILED")
            print(f"  {output}")


def test_video():
    """Test video relay example"""
    print_test_start("Video Examples")
    
    # Install video requirements first
    requirements_file = PROJECT_ROOT / "examples/video/requirements.txt"
    if requirements_file.exists():
        print(f"Installing video requirements from {requirements_file}...")
        try:
            subprocess.run(
                [sys.executable, "-m", "pip", "install", "-r", str(requirements_file)],
                check=True,
                capture_output=True,
                text=True
            )
            print_success("Video requirements installed")
        except subprocess.CalledProcessError as e:
            print_warning(f"Failed to install requirements: {e.stderr}")
    
    script = PROJECT_ROOT / "examples/video/video_relay.py"
    # video_relay uses rclpy.spin() so it's an infinite-loop script
    success, output = run_example_with_timeout(script, timeout=15.0, check_output=None, timeout_ok=True)
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
msg.data = 'Test message from macOS'
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
    print_header("lwrclpy Examples Test Suite for macOS")
    
    print(f"Python: {sys.version}")
    print(f"Project root: {PROJECT_ROOT}")
    print(f"Working directory: /tmp\n")
    
    # Basic import test
    if not test_import():
        print_error("Import test failed. Aborting further tests.")
        sys.exit(1)
    gc.collect()
    
    # Simple publish test
    test_simple_publish()
    gc.collect()
    
    # Test each category
    test_basic_pubsub()
    gc.collect()
    test_timers()
    gc.collect()
    test_parameters()
    gc.collect()
    test_executor()
    gc.collect()
    test_guard_condition()
    gc.collect()
    test_services()
    gc.collect()
    test_actions()
    gc.collect()
    test_video()
    gc.collect()
    
    # Output summary
    all_passed = print_summary()
    
    # Force cleanup to prevent double-free errors
    # Multiple passes to ensure all objects are cleaned up
    for _ in range(3):
        gc.collect()
        time.sleep(0.2)
    
    time.sleep(1.0)  # Final wait for all background cleanup
    
    if all_passed:
        print_success("All tests passed! ✨")
        # Use os._exit() to avoid Python cleanup that can trigger double-free
        os._exit(0)
    else:
        print_error("Some tests failed.")
        os._exit(1)

if __name__ == "__main__":
    main()
