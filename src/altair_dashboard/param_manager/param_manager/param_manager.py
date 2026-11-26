import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import yaml
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rclpy.parameter import Parameter
import json
from std_srvs.srv import Trigger
import subprocess, datetime
from std_msgs.msg import String

class ParamManager(Node):
    def __init__(self):
        super().__init__('param_manager')

        # Declare parameters for package and YAML file
        self.declare_parameter('walk_package', 'quintic_walk')  # Default to quintic_walk
        self.declare_parameter('yaml_file', 'config/walking.yaml')  # Relative path within walk package
        
        walk_package = self.get_parameter('walk_package').value
        yaml_file_name = self.get_parameter('yaml_file').value

        try:
            self.yaml_file = self._resolve_yaml_file_path(walk_package, yaml_file_name)
            self.get_logger().info(f"Resolved YAML file path: {self.yaml_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to resolve YAML file path: {e}")
            raise

        self.params = self.load_yaml(self.yaml_file)
        self.get_logger().info(f"Loaded parameters from {self.yaml_file}")
        # self.get_logger().info(f"Generated json string from parameters: {self.jsonObjectString}")

        # Declare parameters in the ROS 2 parameter server
        self.declare_nested_parameters(self.params)

        self.save_param_service = self.create_service(
            Trigger, 'param_manager/save_parameters', self.save_parameters_callback
        )

        self.get_saved_params = self.create_service(
            Trigger, 'param_manager/get_latest_saved_parameters', self.get_latest_saved_parameters_callback
        )

        self.get_all_history_file = self.create_service(
            Trigger, 'param_manager/get_all_history_file', self.get_all_history_file_callback
        )

        self.load_params_from_file_service = self.create_service(
            Trigger, 'param_manager/load_parameters_from_file', self.load_parameters_from_file_callback
        )
        
        self.get_params_from_file_service = self.create_service(
            Trigger, 'param_manager/get_parameters_from_file', self.get_parameters_from_file_callback
        )

        self.delete_param_file_service = self.create_service(
            Trigger, 'param_manager/delete_parameter_file', self.delete_parameter_file_callback
        )

        self.file_path_subscription = self.create_subscription(
            String,
            'param_manager/file_path',
            self.file_path_callback,
            10
        )
        
        self.file_path_preview_subscription = self.create_subscription(
            String,
            'param_manager/file_path_preview',
            self.file_path_preview_callback,
            10
        )

        self.file_path_delete_subscription = self.create_subscription(
            String,
            'param_manager/file_path_delete',
            self.file_path_delete_callback,
            10
        )
        
        # Store the latest received file paths
        self.latest_file_path = None
        self.latest_preview_file_path = None
        self.latest_delete_file_path = None

        # self.list_param_type_service = self.create_service(
        #     Trigger, '/param_manager/get_parameters_type_json', self.list_parameters_type_callback
        # )
    def delete_parameter_file_callback(self, request, response):
        """Callback for deleting a specified parameter file."""
        try:
            file_path = self.latest_delete_file_path
            
            # Check if the file exists
            if not os.path.isfile(file_path):
                response.success = False
                response.message = f"File not found: {file_path}"
                return response
            
            # Check that we're not deleting the main config file
            if file_path == self.yaml_file:
                response.success = False
                response.message = f"Cannot delete the active configuration file: {file_path}"
                return response
            
            # Delete the file
            os.remove(file_path)

            self.get_logger().info(f"Deleted parameter file: {file_path}")
            
            response.success = True
            response.message = f"Successfully deleted parameter file: {file_path}"
            return response
        except Exception as e:
            self.get_logger().error(f"Error deleting parameter file: {e}")
            response.success = False
            response.message = f"Error deleting file: {str(e)}"
            return response

    def get_latest_saved_parameters_callback(self, request, response):
        """Callback for getting the latest saved parameters."""
        try:
            # Get the base directory and filename without extension
            base_dir = os.path.dirname(self.yaml_file)
            base_filename = os.path.basename(self.yaml_file).replace('.yaml', '')
            
            # Get all backup files that match the pattern
            backup_files = []
            for file in os.listdir(base_dir):
                if file.startswith(base_filename) and file.endswith('.yaml') and file != os.path.basename(self.yaml_file):
                    backup_files.append(os.path.join(base_dir, file))
            
            if not backup_files:
                response.success = False
                response.message = "No backup files found"
                return response
            
            # Sort backup files by modification time (newest first)
            backup_files.sort(key=os.path.getmtime, reverse=True)
            latest_file = backup_files[0]
            
            # Load the parameters from the latest backup file
            latest_params = self.load_yaml(latest_file)
            
            response.success = True
            response.message = json.dumps(latest_params)
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to get latest saved parameters: {e}")
            response.success = False
            response.message = f"Failed to get latest saved parameters: {e}"
            return response

    def get_all_history_file_callback(self, request, response):
        """Callback for getting all history files."""
        try:
            # Get the base directory and filename without extension
            base_dir = os.path.dirname(self.yaml_file)
            base_filename = os.path.basename(self.yaml_file).replace('.yaml', '')
            
            # Get all backup files that match the pattern
            history_files = []
            for file in os.listdir(base_dir):
                if file.startswith(base_filename) and file.endswith('.yaml'):
                    file_path = os.path.join(base_dir, file)
                    # Get file modification time
                    mod_time = os.path.getmtime(file_path)
                    # Format the modification time
                    mod_time_str = datetime.datetime.fromtimestamp(mod_time).strftime("%Y-%m-%d %H:%M:%S")
                    # Add file info to the list
                    history_files.append({
                        "filename": file,
                        "path": file_path,
                        "modified": mod_time_str
                    })
            
            # Sort history files by modification time (newest first)
            history_files.sort(key=lambda x: x["path"], reverse=True)
            
            response.success = True
            response.message = json.dumps(history_files)
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to get history files: {e}")
            response.success = False
            response.message = f"Failed to get history files: {e}"
            return response

    def save_parameters_callback(self, request, response):
        """Callback for saving parameters to the YAML file."""
        try:
            # Create a backup of the current YAML file with a timestamp
            timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
            backup_yaml_file = f"{self.yaml_file.replace('.yaml', '')}-{timestamp}.yaml"
            
            try:
                os.rename(self.yaml_file, backup_yaml_file)
                self.get_logger().info(f"Renamed {self.yaml_file} to {backup_yaml_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to rename YAML file: {e}")
                response.success = False
                response.message = f"Failed to rename YAML file: {e}"
                return response
            
            # Get the current walk package name for the dump command
            walk_package = self.get_parameter('walk_package').value
            # Get the current node namespace
            namespace = self.get_namespace()

            # Dump the current parameters to the YAML file
            command = f"ros2 param dump {namespace}/{walk_package} > {self.yaml_file}"
            try:
                subprocess.run(command, shell=True, check=True)
                self.get_logger().info(f"Executed command: {command}")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to execute command '{command}': {e}")
                response.success = False
                response.message = f"Failed to save parameters: {e}"
                return response
            
            response.success = True
            response.message = f"Saved parameters to {self.yaml_file}"
            return response
        except Exception as e:
            self.get_logger().error(f"Unexpected error saving parameters: {e}")
            response.success = False
            response.message = f"Unexpected error saving parameters: {e}"
            return response

    def file_path_callback(self, msg):
        """Store the latest file path for loading parameters."""
        self.latest_file_path = msg.data
        self.get_logger().info(f"Received file path for loading: {self.latest_file_path}")

    def file_path_preview_callback(self, msg):
        """Store the latest file path for previewing parameters."""
        self.latest_preview_file_path = msg.data
        self.get_logger().info(f"Received file path for preview: {self.latest_preview_file_path}")

    def file_path_delete_callback(self, msg):
        """Store the latest file path for deleting parameters."""
        self.latest_delete_file_path = msg.data
        self.get_logger().info(f"Received file path for delete: {self.latest_delete_file_path}")

    # Then modify the service callbacks to use these stored paths
    def load_parameters_from_file_callback(self, request, response):
        """Callback for loading parameters from a specific file."""
        try:
            file_path = self.latest_file_path
            
            if not file_path:
                response.success = False
                response.message = "No file path provided"
                return response

            # Check if the file exists
            if not os.path.isfile(file_path):
                response.success = False
                response.message = f"File not found: {file_path}"
                return response
            
            # Load parameters from the file
            loaded_params = self.load_yaml(file_path)
            if not loaded_params:
                response.success = False
                response.message = f"Failed to load parameters from {file_path}"
                return response
            
            # Update parameters in ROS parameter server
            for param_name, value in self._flatten_params(loaded_params):
                param_type = self._infer_parameter_type(value)
                self.set_parameters([Parameter(param_name, param_type, value)])
                self.get_logger().info(f"Set parameter {param_name} to {value}")
            
            # Update the in-memory parameters
            self.params = loaded_params
            
            response.success = True
            response.message = f"Loaded parameters from {file_path}"
            return response
        except Exception as e:
            self.get_logger().error(f"Error loading parameters from file: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response


    def get_parameters_from_file_callback(self, request, response):
        """Callback for getting parameters from a specific file without loading them."""
        try:
            file_path = self.latest_preview_file_path
            
            if not file_path:
                response.success = False
                response.message = "No file path provided"
                return response

            # Check if the file exists
            if not os.path.isfile(file_path):
                response.success = False
                response.message = f"File not found: {file_path}"
                return response
            
            # Load parameters from the file
            loaded_params = self.load_yaml(file_path)
            if not loaded_params:
                response.success = False
                response.message = f"Failed to load parameters from {file_path}"
                return response
            
            # Convert the parameters to JSON and return
            response.success = True
            response.message = json.dumps(loaded_params)
            return response
        except Exception as e:
            self.get_logger().error(f"Error getting parameters from file: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
            return response


    def load_yaml(self, file_path):
        """Load parameters from a YAML file."""
        try:
            with open(file_path, 'r') as file:
                return yaml.safe_load(file) or {}
        except FileNotFoundError:
            self.get_logger().error(f"YAML file {file_path} not found.")
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file {file_path}: {e}")
            return {}

    def save_yaml(self, file_path, data):
        """Save parameters to a YAML file."""
        try:
            with open(file_path, 'w') as file:
                yaml.dump(data, file)
            self.get_logger().info(f"Saved parameters to {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save YAML file {file_path}: {e}")

    def declare_nested_parameters(self, params, prefix=""):
        """Recursively declare nested parameters."""
        for key, value in params.items():
            full_name = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                self.declare_nested_parameters(value, full_name)
            else:
                self.declare_parameter(full_name, value)

    def set_parameters_callback(self, request, response):
        """Callback for setting parameters."""
        for param in request.parameters:
            # Extract the value from the ParameterValue message
            if param.value.type == ParameterType.PARAMETER_BOOL:
                value = param.value.bool_value
                param_type = Parameter.Type.BOOL
            elif param.value.type == ParameterType.PARAMETER_INTEGER:
                value = param.value.integer_value
                param_type = Parameter.Type.INTEGER
            elif param.value.type == ParameterType.PARAMETER_DOUBLE:
                value = param.value.double_value
                param_type = Parameter.Type.DOUBLE
            elif param.value.type == ParameterType.PARAMETER_STRING:
                value = param.value.string_value
                param_type = Parameter.Type.STRING
            elif param.value.type == ParameterType.PARAMETER_STRING_ARRAY:
                value = param.value.string_array_value
                param_type = Parameter.Type.STRING_ARRAY
            else:
                self.get_logger().warn(f"Unsupported parameter type: {param.value.type}")
                continue

            # Update in-memory params
            self._update_nested_params(self.params, param.name, value)

            # Update ROS 2 parameter server
            self.set_parameters([Parameter(param.name, param_type, value)])

            self.get_logger().info(f"Set parameter {param.name} to {value}")

        # Save updated parameters to the YAML file
        self.save_yaml(self.yaml_file, self.params)
        response.results = [self._create_set_parameter_result(True) for _ in request.parameters]
        return response

    def _update_nested_params(self, params, full_name, value):
        """Update nested parameters in the in-memory dictionary."""
        keys = full_name.split('.')
        current = params
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        current[keys[-1]] = value

    def get_parameters_callback(self, request, response):
        """Callback for getting parameters."""
        response.values = []
        for name in request.names:
            if self._has_nested_param(self.params, name):
                param_value = self.get_parameter(name).value
                response.values.append(self._create_parameter_value(param_value))
            else:
                self.get_logger().warn(f"Parameter {name} not found.")
                response.values.append(self._create_parameter_value(None))
        return response

    def _has_nested_param(self, params, full_name):
        """Check if a nested parameter exists."""
        keys = full_name.split('.')
        current = params
        for key in keys:
            if key not in current:
                return False
            current = current[key]
        return True

    # def list_parameters_callback(self, request, response):
    #     """Callback for listing all parameters."""
    #     response.result.names = self._flatten_nested_params(self.params)
    #     return response

    def list_parameters_callback(self, request, response):
        """Callback for listing all parameters in their nested structure."""
        response.success = True
        response.message = json.dumps(self.params)
        return response

    def list_parameters_type_callback(self, request, response):
        """Callback for listing parameter types in their nested structure."""
        param_types = self._get_parameter_types(self.params)
        json_types = json.dumps(param_types)
        response.success = True
        response.message = json_types
        return response

    def _get_parameter_types(self, params):
        """Recursively build a nested structure of parameter types."""
        param_types = {}
        for key, value in params.items():
            if isinstance(value, dict):
                param_types[key] = self._get_parameter_types(value)
            else:
                param_type = self._infer_parameter_type(value)
                param_types[key] = self._parameter_type_to_string(param_type)
        return param_types

    def _parameter_type_to_string(self, param_type):
        """Convert ROS 2 ParameterType to human-readable string."""
        type_map = {
            ParameterType.PARAMETER_BOOL: "bool",
            ParameterType.PARAMETER_INTEGER: "int",
            ParameterType.PARAMETER_DOUBLE: "double",
            ParameterType.PARAMETER_STRING: "string",
            ParameterType.PARAMETER_STRING_ARRAY: "string_array",
            ParameterType.PARAMETER_NOT_SET: "not_set",
        }
        return type_map.get(param_type, "unknown")

    def _flatten_nested_params(self, params, prefix=""):
        """Flatten nested parameters into a list of full names."""
        names = []
        for key, value in params.items():
            full_name = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                names.extend(self._flatten_nested_params(value, full_name))
            else:
                names.append(full_name)
        return names

    def _create_set_parameter_result(self, success):
        """Helper to create a SetParametersResult message."""
        from rcl_interfaces.msg import SetParametersResult
        result = SetParametersResult()
        result.successful = success
        return result

    def _create_parameter_value(self, value):
        """Helper to create a ParameterValue message."""
        from rcl_interfaces.msg import ParameterValue
        param_value = ParameterValue()
        param_type = self._infer_parameter_type(value)

        # Set the appropriate field based on the parameter type
        if param_type == ParameterType.PARAMETER_BOOL:
            param_value.bool_value = value
        elif param_type == ParameterType.PARAMETER_INTEGER:
            param_value.integer_value = value
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            param_value.double_value = value
        elif param_type == ParameterType.PARAMETER_STRING:
            param_value.string_value = value
        elif param_type == ParameterType.PARAMETER_STRING_ARRAY:
            param_value.string_array_value = value
        else:
            self.get_logger().warn(f"Unsupported parameter type for value: {value}")
            param_value.type = ParameterType.PARAMETER_NOT_SET

        param_value.type = param_type
        return param_value

    def _infer_parameter_type(self, value):
        """Infer the ROS 2 parameter type from a Python value."""
        if isinstance(value, bool):
            return ParameterType.PARAMETER_BOOL
        elif isinstance(value, int):
            return ParameterType.PARAMETER_INTEGER
        elif isinstance(value, float):
            return ParameterType.PARAMETER_DOUBLE
        elif isinstance(value, str):
            return ParameterType.PARAMETER_STRING
        elif isinstance(value, list):
            return ParameterType.PARAMETER_STRING_ARRAY  # Adjust based on your use case
        else:
            return ParameterType.PARAMETER_NOT_SET

    def _resolve_yaml_file_path(self, package_name, yaml_file_name):
        """Resolve the path to the YAML file for the specified package."""
        try:
            package_share_dir = get_package_share_directory(package_name)
            path_string = os.path.join(package_share_dir, yaml_file_name)
            
            # Map package names to their source directories
            package_source_map = {
                'quintic_walk': '/src/altair_motion/quintic_walk',
                'cubic_walk': '/src/altair_motion/cubic_walk',
                'b_splines_walk': '/src/altair_motion/b_splines_walk'
            }
            
            if package_name in package_source_map:
                # Replace install path with source path
                install_path = f'/install/{package_name}/share/{package_name}'
                source_path = package_source_map[package_name]
                yaml_file_path = path_string.replace(install_path, source_path)
            else:
                self.get_logger().warn(f"Unknown package {package_name}, using install path")
                yaml_file_path = path_string
            
            return yaml_file_path
        except Exception as e:
            raise Exception(f"Failed to resolve path for package {package_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ParamManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()