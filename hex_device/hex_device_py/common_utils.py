#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Jecjune. All rights reserved.
# Author: Jecjune zejun.chen@hexfellow.com
# Date  : 2025-8-1
################################################################
import time
import asyncio
import logging


async def delay(start_time, ms):
    end_time = start_time + ms / 1000
    now = time.perf_counter()
    sleep_time = end_time - now
    # Handle negative delay (when we're already past the target time)
    if sleep_time <= 0:
        # Log warning if delay is significantly negative (> 1ms)
        if sleep_time < -0.001:
            log_debug(f"HexDevice: Negative delay detected: {sleep_time*1000:.2f}ms - cycle overrun")
        return  # Don't sleep if we're already late
    
    await asyncio.sleep(sleep_time)

# Create a logger for the hex_device package
_logger = logging.getLogger(__name__.split('.')[0])  # Use 'hex_device_py' as logger name

# ANSI color codes for terminal output
class LogColors:
    """ANSI color codes for different log levels"""
    RESET = '\033[0m'
    DEBUG = '\033[36m'      # Cyan
    WARNING = '\033[33m'    # Yellow
    ERROR = '\033[31m'      # Red
    CRITICAL = '\033[35m'   # Magenta

class ColoredFormatter(logging.Formatter):
    """Custom formatter with colors for different log levels"""

    COLORS = {
        logging.DEBUG: LogColors.DEBUG,
        logging.INFO: None,  # No color for INFO
        logging.WARNING: LogColors.WARNING,
        logging.ERROR: LogColors.ERROR,
        logging.CRITICAL: LogColors.CRITICAL,
    }

    def format(self, record):
        color = self.COLORS.get(record.levelno)

        # Format the base message
        log_fmt = f'{color}[%(levelname)s] %(message)s{LogColors.RESET}' if color else '[%(levelname)s] %(message)s'
        formatter = logging.Formatter(log_fmt)
        formatted = formatter.format(record)

        # Handle multiline: add color to each continuation line
        if color and '\n' in formatted:
            lines = formatted.split('\n')
            colored_lines = []
            for line in lines:
                if line:  # Skip empty lines
                    colored_lines.append(f"{color}{line}{LogColors.RESET}")
                else:
                    colored_lines.append(line)
            return '\n'.join(colored_lines)

        return formatted

# Configure logger to output to console
if not _logger.handlers:  # Only configure once
    _logger.setLevel(logging.DEBUG)  # Set minimum level to DEBUG

    # Create console handler
    _console_handler = logging.StreamHandler()
    _console_handler.setLevel(logging.DEBUG)

    # Create colored formatter
    _formatter = ColoredFormatter()
    _console_handler.setFormatter(_formatter)

    # Add handler to logger
    _logger.addHandler(_console_handler)

def log_warn(message):
    """Log warning message"""
    _logger.warning(message)

def log_err(message):
    """Log error message"""
    _logger.error(message)

def log_info(message):
    """Log info message"""
    _logger.info(message)

def log_common(message):
    """Log common message (info level)"""
    _logger.info(message)

def log_debug(message):
    """Log debug message"""
    _logger.debug(message)
