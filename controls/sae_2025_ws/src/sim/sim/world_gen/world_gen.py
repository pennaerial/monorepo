from abc import ABC, abstractmethod
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom


class WorldGenerator(ABC):
    """
    Abstract base class for world generation in different competitions.
    This class provides the basic structure and common functionality for generating
    competition worlds while allowing specific implementations to define their own
    course types and generation logic.
    """
    
    def __init__(self, input_file: str, output_file: str):
        """
        Initialize the world generator with input and output file paths.
        
        Args:
            input_file (str): Path to the input SDF file
            output_file (str): Path where the generated world will be saved
        """
        self.input_file = input_file
        self.output_file = output_file

    def generate_world(self, style: str, params: dict) -> None:
        """
        Main entry point for world generation. Handles the overall flow while
        letting subclasses implement specific generation logic.
        
        Args:
            style (str): The style/type of course to generate
            params (dict): Parameters for world generation
        """
        # Validate input files
        self._validate_files()
        
        # Generate the course based on style
        course_elements = self._generate_course(style, params)
        
        # Add elements to the world
        self._add_to_world(course_elements)

    @abstractmethod
    def _generate_course(self, style: str, params: dict) -> list:
        """
        Abstract method for generating course-specific elements.
        Must be implemented by subclasses to define specific course generation logic.
        
        Args:
            style (str): The style/type of course to generate
            params (dict): Parameters for course generation
            
        Returns:
            list: List of course elements to be added to the world
        """
        pass

    @abstractmethod
    def _add_to_world(self, elements: list) -> None:
        """
        Abstract method for adding generated elements to the world.
        Must be implemented by subclasses to define how elements are added to the SDF.
        
        Args:
            elements (list): List of elements to add to the world
        """
        pass

    def _validate_files(self) -> None:
        """
        Validates input and output file paths.
        """
        in_path = Path(self.input_file).expanduser().resolve()
        out_path = Path(self.output_file).expanduser().resolve()
        
        if not in_path.exists():
            raise FileNotFoundError(
                f"Input SDF not found: {in_path}\nCWD: {Path.cwd().resolve()}"
            )
        out_path.parent.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _format_sdf(root: ET.Element) -> str:
        """
        Helper method to format SDF XML with proper indentation.
        
        Args:
            root (ET.Element): Root element of the SDF XML
            
        Returns:
            str: Formatted XML string
        """
        # Remove whitespace to avoid extra blank lines
        def strip_whitespace(elem):
            if elem.text is not None and elem.text.strip() == "":
                elem.text = None
            for child in list(elem):
                strip_whitespace(child)
                if child.tail is not None and child.tail.strip() == "":
                    child.tail = None
        
        strip_whitespace(root)
        
        # Format the XML
        rough_bytes = ET.tostring(root, encoding="utf-8")
        pretty = minidom.parseString(rough_bytes.decode("utf-8")).toprettyxml(indent="  ")
        lines = [ln for ln in pretty.splitlines() if ln.strip() != ""]
        return "\n".join(lines) + "\n"