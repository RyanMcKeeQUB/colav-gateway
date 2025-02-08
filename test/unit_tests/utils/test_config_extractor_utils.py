#!/usr/bin/env python3
"""
Unit tests for config_extractor utils
"""
from utils.config_extractor_utils import extract_endpoint, EndpointEnum
import unittest

class TestConfigExtractorUtils(unittest.TestCase):

    def setUp(self):
        # Sample data in dictionary format, simulating the YAML structure.
        self.config_data = {
            "mission_request": {"host": "0.0.0.0", "port": 7000},
            "mission_response": {"host": "0.0.0.0", "port": 7001},
            "agent_config": {"host": "0.0.0.0", "port": 7100},
            "obstacles_config": {"host": "0.0.0.0", "port": 7200},
            "controller_feedback": {"host": "0.0.0.0", "port": 7300}
        }

    def test_extract_endpoints(self):
        # Iterate over each config type in self.config_data
        for config_name, config_info in self.config_data.items():
            # Map the config name to the corresponding EndpointEnum
            key = EndpointEnum[config_name.upper()]  # Converts to proper enum key
            
            with self.subTest(config_name=config_name):
                # Simulate the data extraction
                extracted_data = extract_endpoint(endpoint_config=self.config_data, key=key)
                
                # Check if the extraction is correct (extracting host and port)
                self.assertEqual(extracted_data[0], config_info['host'])
                self.assertEqual(extracted_data[1], config_info['port'])

if __name__ == "__main__":
    unittest.main()
