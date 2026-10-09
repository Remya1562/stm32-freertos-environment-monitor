
import csv
import re
import time
from pathlib import Path
from datetime import datetime

import serial
from serial import SerialException


# ============================================================
# CONFIGURATION
# ============================================================

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1

# Both timeouts are 8 seconds
UART_TIMEOUT = 8
LIVE_SENSOR_TIMEOUT = 8

PROJECT_DIR = Path(
    r"C:\Users\Ravi Sankar\Desktop\breathecode\mini project 1"
    r"\miniproject1_rtos_version"
)

REPORT_DIR = PROJECT_DIR / "test_reports"
REPORT_DIR.mkdir(parents=True, exist_ok=True)

REPORT_FILE = REPORT_DIR / "environment_test_report.csv"


# ============================================================
# REGULAR EXPRESSIONS
# ============================================================

DATA_PATTERN = re.compile(
    r"Temp\s*=\s*(-?\d+(?:\.\d+)?)\s*°?\s*C\s*,\s*"
    r"Hum\s*=\s*(-?\d+(?:\.\d+)?)\s*%\s*,\s*"
    r"Threshold\s*=\s*(-?\d+(?:\.\d+)?)"
    r"\s*(?:°\s*C|C)?",
    re.IGNORECASE
)

THRESHOLD_PATTERN = re.compile(
    r"Adjusting threshold:\s*(-?\d+(?:\.\d+)?)",
    re.IGNORECASE
)

MODE_PATTERN = re.compile(
    r"Mode switched:\s*(LIVE_STREAMING|THRESHOLD_ADJUST)",
    re.IGNORECASE
)


# ============================================================
# TEST CASE DEFINITIONS
# ============================================================

TESTS = {
    "TC01": {
        "description": "Temperature data received",
        "expected": "Numeric temperature value received",
        "observed": "",
        "actual": "No valid reading received yet",
        "status": "NOT RUN",
    },
    "TC02": {
        "description": "Temperature range validation",
        "expected": "-40 to 85 C",
        "observed": "",
        "actual": "No valid reading received yet",
        "status": "NOT RUN",
    },
    "TC03": {
        "description": "Humidity data received",
        "expected": "Numeric humidity value received",
        "observed": "",
        "actual": "No valid reading received yet",
        "status": "NOT RUN",
    },
    "TC04": {
        "description": "Humidity range validation",
        "expected": "0 to 100 %",
        "observed": "",
        "actual": "No valid reading received yet",
        "status": "NOT RUN",
    },
    "TC05": {
        "description": "Threshold data received",
        "expected": "Numeric threshold value received",
        "observed": "",
        "actual": "No valid reading received yet",
        "status": "NOT RUN",
    },
    "TC06": {
        "description": "Threshold range validation",
        "expected": "25 to 45 C",
        "observed": "",
        "actual": "No valid reading received yet",
        "status": "NOT RUN",
    },
    "ADJ01": {
        "description": "Threshold adjustment range validation",
        "expected": "25 to 45 C",
        "observed": "",
        "actual": "No threshold adjustment received yet",
        "status": "NOT RUN",
    },
    "UART01": {
        "description": "UART communication timeout",
        "expected": "UART messages received within 8 seconds",
        "observed": "",
        "actual": "Timeout test not triggered",
        "status": "NOT RUN",
    },
    "UART02": {
        "description": "Live sensor data timeout",
        "expected": "Valid sensor data received within 8 seconds",
        "observed": "",
        "actual": "Timeout test not triggered",
        "status": "NOT RUN",
    },
}


REQUIRED_SENSOR_TESTS = [
    "TC01", "TC02", "TC03", "TC04", "TC05", "TC06"
]


# ============================================================
# SUMMARY CALCULATION
# ============================================================

def calculate_summary(final=False):
    statuses = [test["status"] for test in TESTS.values()]

    failed_count = statuses.count("FAIL")
    passed_count = statuses.count("PASS")

    if failed_count:
        return (
            "FAIL",
            f"{passed_count} passed, {failed_count} failed"
        )

    if not final:
        return (
            "IN PROGRESS",
            f"{passed_count} passed, {failed_count} failed"
        )

    missing_required = [
        test_id
        for test_id in REQUIRED_SENSOR_TESTS
        if TESTS[test_id]["status"] == "NOT RUN"
    ]

    if missing_required:
        return (
            "INCOMPLETE",
            "Required tests not executed: " + ", ".join(missing_required)
        )

    return (
        "PASS",
        "All required sensor tests passed; no failures recorded"
    )


# ============================================================
# CSV REPORT
# One row per test case and one SUMMARY row
# ============================================================

def write_report(final=False):
    summary_status, summary_actual = calculate_summary(final)

    columns = [
        "Timestamp",
        "Test Case ID",
        "Test Description",
        "Input / Observed Value",
        "Expected Result",
        "Actual Result",
        "Status",
    ]

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    with open(
        REPORT_FILE,
        mode="w",
        newline="",
        encoding="utf-8"
    ) as report_file:

        writer = csv.DictWriter(
            report_file,
            fieldnames=columns
        )
        writer.writeheader()

        for test_id, result in TESTS.items():
            writer.writerow({
                "Timestamp": timestamp,
                "Test Case ID": test_id,
                "Test Description": result["description"],
                "Input / Observed Value": result["observed"],
                "Expected Result": result["expected"],
                "Actual Result": result["actual"],
                "Status": result["status"],
            })

        writer.writerow({
            "Timestamp": timestamp,
            "Test Case ID": "SUMMARY",
            "Test Description": "Overall test session result",
            "Input / Observed Value": (
                f"{sum(t['status'] == 'PASS' for t in TESTS.values())} passed, "
                f"{sum(t['status'] == 'FAIL' for t in TESTS.values())} failed"
            ),
            "Expected Result": "All required sensor tests pass",
            "Actual Result": summary_actual,
            "Status": summary_status,
        })


def update_test(test_id, observed, actual, status):
    TESTS[test_id]["observed"] = observed
    TESTS[test_id]["actual"] = actual
    TESTS[test_id]["status"] = status
    write_report()


# Write an initial report
write_report()


# ============================================================
# SENSOR TEST CASES
# ============================================================

def run_sensor_tests(temperature, humidity, threshold):

    # TC01: Temperature received
    update_test(
        "TC01",
        f"{temperature:.2f} C",
        "Numeric temperature received",
        "PASS"
    )

    # TC02: Temperature range
    valid = -40 <= temperature <= 85
    update_test(
        "TC02",
        f"{temperature:.2f} C",
        "Temperature within range" if valid
        else "Temperature outside range",
        "PASS" if valid else "FAIL"
    )

    # TC03: Humidity received
    update_test(
        "TC03",
        f"{humidity:.2f} %",
        "Numeric humidity received",
        "PASS"
    )

    # TC04: Humidity range
    valid = 0 <= humidity <= 100
    update_test(
        "TC04",
        f"{humidity:.2f} %",
        "Humidity within range" if valid
        else "Humidity outside range",
        "PASS" if valid else "FAIL"
    )

    # TC05: Threshold received
    update_test(
        "TC05",
        f"{threshold:.2f} C",
        "Numeric threshold received",
        "PASS"
    )

    # TC06: Threshold range
    valid = 25 <= threshold <= 45
    update_test(
        "TC06",
        f"{threshold:.2f} C",
        "Threshold within range" if valid
        else "Threshold outside range",
        "PASS" if valid else "FAIL"
    )


# ============================================================
# MAIN UART LOOP
# ============================================================

def main():

    current_mode = "LIVE_STREAMING"

    last_uart_time = time.monotonic()
    last_sensor_time = None

    uart_timeout_logged = False
    sensor_timeout_logged = False

    try:
        with serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=SERIAL_TIMEOUT,
        ) as ser:

            while True:

                raw_line = ser.readline()

                if raw_line:
                    line = raw_line.decode(
                        "utf-8",
                        errors="replace"
                    ).strip()

                    if line:
                        # Print only raw STM32 UART output.
                        print(line, flush=True)

                        now = time.monotonic()
                        last_uart_time = now

                        # UART communication resumed.
                        if uart_timeout_logged:
                            update_test(
                                "UART01",
                                "UART message received",
                                "UART communication resumed",
                                "PASS"
                            )
                            uart_timeout_logged = False

                        # ------------------------------------
                        # MODE DETECTION
                        # ------------------------------------

                        mode_match = MODE_PATTERN.search(line)

                        if mode_match:
                            current_mode = mode_match.group(1).upper()

                            if current_mode == "LIVE_STREAMING":
                                last_sensor_time = None
                                sensor_timeout_logged = False

                            continue

                        # ------------------------------------
                        # THRESHOLD ADJUSTMENT
                        # ------------------------------------

                        threshold_match = THRESHOLD_PATTERN.search(line)

                        if threshold_match:
                            threshold = float(threshold_match.group(1))
                            valid = 25 <= threshold <= 45

                            update_test(
                                "ADJ01",
                                f"{threshold:.2f} C",
                                "Adjusted threshold within range" if valid
                                else "Adjusted threshold outside range",
                                "PASS" if valid else "FAIL"
                            )
                            continue

                        # ------------------------------------
                        # SENSOR DATA
                        # ------------------------------------

                        data_match = DATA_PATTERN.search(line)

                        if data_match:
                            temperature = float(data_match.group(1))
                            humidity = float(data_match.group(2))
                            threshold = float(data_match.group(3))

                            last_sensor_time = time.monotonic()
                            sensor_timeout_logged = False

                            # Sensor data resumed after a timeout.
                            if TESTS["UART02"]["status"] == "FAIL":
                                update_test(
                                    "UART02",
                                    "Valid sensor reading received",
                                    "Live sensor data resumed",
                                    "PASS"
                                )

                            run_sensor_tests(
                                temperature,
                                humidity,
                                threshold
                            )

                # --------------------------------------------
                # UART COMMUNICATION TIMEOUT: 8 SECONDS
                # --------------------------------------------

                now = time.monotonic()

                if now - last_uart_time >= UART_TIMEOUT:
                    if not uart_timeout_logged:
                        update_test(
                            "UART01",
                            f"No UART message for {UART_TIMEOUT} seconds",
                            "UART communication timeout",
                            "FAIL"
                        )
                        uart_timeout_logged = True

                # --------------------------------------------
                # LIVE SENSOR TIMEOUT: 8 SECONDS
                # UART messages may still be arriving.
                # --------------------------------------------

                if current_mode == "LIVE_STREAMING":
                    if last_sensor_time is not None:
                        elapsed = now - last_sensor_time

                        if elapsed >= LIVE_SENSOR_TIMEOUT:
                            if not sensor_timeout_logged:
                                update_test(
                                    "UART02",
                                    f"No valid sensor data for {elapsed:.1f} seconds",
                                    "Live sensor data timeout",
                                    "FAIL"
                                )
                                sensor_timeout_logged = True

    except KeyboardInterrupt:
        pass

    except (SerialException, OSError) as exc:
        update_test(
            "UART01",
            SERIAL_PORT,
            f"Serial connection error: {exc}",
            "FAIL"
        )

    except Exception as exc:
        update_test(
            "UART01",
            "Python UART test",
            f"Unexpected error: {exc}",
            "FAIL"
        )

    finally:
        write_report(final=True)


if __name__ == "__main__":
    main()
