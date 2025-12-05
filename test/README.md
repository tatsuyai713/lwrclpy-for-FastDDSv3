# lwrclpy Examples Test Suite for macOS and Ubuntu

This directory contains test scripts for verifying that lwrclpy sample code runs correctly on macOS and Ubuntu.

## Test Scripts

- `test_examples_mac.py` - Test suite for macOS
- `test_examples_ubuntu.py` - Test suite for Ubuntu/Linux

Both scripts test the same examples but are optimized for their respective platforms.

## Requirements

### macOS
- macOS
- Python 3.11 or higher
- Virtual environment with lwrclpy installed

### Ubuntu
- Ubuntu 20.04 or higher
- Python 3.8 or higher
- Virtual environment with lwrclpy installed

## How to Run Tests

### macOS

#### 1. Activate virtual environment

```bash
source ~/venv/bin/activate
```

#### 2. Run test script

```bash
cd /Users/tatsuyai/repos/lwrclpy-for-FastDDSv3
python3 test/test_examples_mac.py
```

Or

```bash
./test/test_examples_mac.py
```

### Ubuntu

#### 1. Activate virtual environment

```bash
source ~/venv/bin/activate
```

#### 2. Run test script

```bash
cd /path/to/lwrclpy-for-FastDDSv3
python3 test/test_examples_ubuntu.py
```

Or

```bash
./test/test_examples_ubuntu.py
```

## Test Coverage

The test script tests sample code in the following categories sequentially:

### 1. Import Test
- Basic import of lwrclpy and message types

### 2. Simple Publish Test
- Basic operations: node creation, publisher creation, message publishing

### 3. Basic Publisher/Subscriber
- `examples/pubsub/string/talker.py`
- `examples/pubsub/string/listener.py`

### 4. Timers
- `examples/timers/wall_timer.py`
- `examples/timers/wall_timer_listener.py`
- `examples/timers/oneshot_and_periodic.py`
- `examples/timers/oneshot_and_periodic_listener.py`

### 5. Parameters
- `examples/parameters/logger_and_params.py`

### 6. Executors
- `examples/executor/single_node_spin.py`
- `examples/executor/multithreaded_spin.py`

### 7. Guard Condition
- `examples/guard_condition/trigger_guard_condition.py`

### 8. Services
- `examples/services/set_bool/server.py`
- `examples/services/set_bool/client.py`
- `examples/services/trigger_bridge/bridge.py`

### 9. Actions
- `examples/actions/fibonacci_action_server.py`
- `examples/actions/fibonacci_action_client.py`

## Understanding Test Results

- ✓ (green): Test passed
- ✗ (red): Test failed
- ⚠ (yellow): Warning

At the end, a test summary displays the number of passed tests, failed tests, and success rate.

## Troubleshooting

### Import Errors

Check if lwrclpy is correctly installed:

```bash
pip list | grep lwrclpy
```

To reinstall:

```bash
pip uninstall -y lwrclpy
pip install /path/to/lwrclpy-0.1.1-cp311-cp311-macosx_26_0_universal2.whl
```

### Cannot Run from Project Directory

The test script automatically executes examples from the `/tmp` directory. This ensures the installed lwrclpy package is used instead of the source code.

### Timeout Errors

The default timeout is 3 seconds. You can adjust the `timeout` parameter in the script.

## Notes

- Each test runs independently and does not affect others
- Video-related samples (`examples/video/`) have many dependencies and are not included in this test script
- Service and action tests only perform basic startup tests since server/client pairs are not run simultaneously

## Continuous Integration

These test scripts are used in GitHub Actions CI/CD pipeline:

- **Ubuntu CI**: Runs `test_examples_ubuntu.py` on Ubuntu 20.04/22.04
- **macOS CI**: Runs `test_examples_mac.py` on macOS latest
- **Build CI**: Creates wheel packages for both platforms after tests pass

See `.github/workflows/` directory for CI configuration details.

## Test Results Interpretation

All tests should pass (100% success rate) for a successful build:

- **Import Test**: Verifies basic lwrclpy functionality
- **Simple Publish Test**: Tests node creation and message publishing
- **Example Tests**: Each example should run without errors or timeouts

If any test fails, check:
1. Library dependencies are installed correctly
2. FastDDS v3 is properly configured
3. Python environment is set up correctly

