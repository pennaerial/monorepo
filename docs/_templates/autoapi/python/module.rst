{# =====================================================================
   Override of sphinx-autoapi's built-in templates/python/module.rst.

   Why this file exists at all:
   AutoAPI loads templates with a plain jinja2 FileSystemLoader that
   checks `autoapi_template_dir` first and falls back to its own
   built-in templates dir. There's no way to "extend" or patch just one
   block of the built-in template from here - putting a file at this
   same relative path (python/module.rst) fully replaces the built-in
   one. So this is a full copy of the upstream template with a couple
   deliberate changes (marked "CHANGED" below), not a diff/patch file.

   If sphinx-autoapi is upgraded and the sidebar/headings look wrong
   again, diff this against the new package's
   autoapi/templates/python/module.rst and re-apply the same changes.

   Changes made, both switching from obj.id (fully-qualified dotted
   name, e.g. "uav.modes.LandingMode") to obj.short_name (last
   component only, e.g. "LandingMode"):
   1. The "Submodules" toctree entries used to be bare paths. Sphinx
      then falls back to using each target page's own title as the
      link text, which is why the sidebar showed a repeated
      "uav" > "uav.modes" > "uav.modes.LandingMode" chain instead of a
      normal collapsing tree.
   2. The page's own top heading, for the same reason - it was also
      rendering the fully-qualified name.
   The `.. py:module::` directives are left as obj.name (fully
   qualified) since Sphinx needs that internally for cross-references
   and permalinks to keep resolving correctly.
   ===================================================================== #}
{% if obj.display %}
   {% if is_own_page %}
{# CHANGED: obj.id -> obj.short_name #}
{{ obj.short_name }}
{{ "=" * obj.short_name|length }}

.. py:module:: {{ obj.name }}

      {% if obj.docstring %}
.. autoapi-nested-parse::

   {{ obj.docstring|indent(3) }}

      {% endif %}

      {% block submodules %}
         {% set visible_subpackages = obj.subpackages|selectattr("display")|list %}
         {% set visible_submodules = obj.submodules|selectattr("display")|list %}
         {% set visible_submodules = (visible_subpackages + visible_submodules)|sort %}
         {% if visible_submodules %}
Submodules
----------

.. toctree::
   :maxdepth: 1

            {% for submodule in visible_submodules %}
   {# CHANGED: explicit short title instead of a bare path #}
   {{ submodule.short_name }} <{{ submodule.include_path }}>
            {% endfor %}


         {% endif %}
      {% endblock %}
      {% block content %}
         {% set visible_children = obj.children|selectattr("display")|list %}
         {% if visible_children %}
            {% set visible_attributes = visible_children|selectattr("type", "equalto", "data")|list %}
            {% if visible_attributes %}
               {% if "attribute" in own_page_types or "show-module-summary" in autoapi_options %}
Attributes
----------

                  {% if "attribute" in own_page_types %}
.. toctree::
   :hidden:

                     {% for attribute in visible_attributes %}
   {{ attribute.include_path }}
                     {% endfor %}

                  {% endif %}
.. autoapisummary::

                  {% for attribute in visible_attributes %}
   {{ attribute.id }}
                  {% endfor %}
               {% endif %}


            {% endif %}
            {% set visible_exceptions = visible_children|selectattr("type", "equalto", "exception")|list %}
            {% if visible_exceptions %}
               {% if "exception" in own_page_types or "show-module-summary" in autoapi_options %}
Exceptions
----------

                  {% if "exception" in own_page_types %}
.. toctree::
   :hidden:

                     {% for exception in visible_exceptions %}
   {{ exception.include_path }}
                     {% endfor %}

                  {% endif %}
.. autoapisummary::

                  {% for exception in visible_exceptions %}
   {{ exception.id }}
                  {% endfor %}
               {% endif %}


            {% endif %}
            {% set visible_classes = visible_children|selectattr("type", "equalto", "class")|list %}
            {% if visible_classes %}
               {% if "class" in own_page_types or "show-module-summary" in autoapi_options %}
Classes
-------

                  {% if "class" in own_page_types %}
.. toctree::
   :hidden:

                     {% for klass in visible_classes %}
   {{ klass.include_path }}
                     {% endfor %}

                  {% endif %}
.. autoapisummary::

                  {% for klass in visible_classes %}
   {{ klass.id }}
                  {% endfor %}
               {% endif %}


            {% endif %}
            {% set visible_functions = visible_children|selectattr("type", "equalto", "function")|list %}
            {% if visible_functions %}
               {% if "function" in own_page_types or "show-module-summary" in autoapi_options %}
Functions
---------

                  {% if "function" in own_page_types %}
.. toctree::
   :hidden:

                     {% for function in visible_functions %}
   {{ function.include_path }}
                     {% endfor %}

                  {% endif %}
.. autoapisummary::

                  {% for function in visible_functions %}
   {{ function.id }}
                  {% endfor %}
               {% endif %}


            {% endif %}
            {% set this_page_children = visible_children|rejectattr("type", "in", own_page_types)|list %}
            {% if this_page_children %}
{{ obj.type|title }} Contents
{{ "-" * obj.type|length }}---------

               {% for obj_item in this_page_children %}
{{ obj_item.render()|indent(0) }}
               {% endfor %}
            {% endif %}
         {% endif %}
      {% endblock %}
   {% else %}
.. py:module:: {{ obj.name }}

      {% if obj.docstring %}
   .. autoapi-nested-parse::

      {{ obj.docstring|indent(6) }}

      {% endif %}
      {% for obj_item in visible_children %}
   {{ obj_item.render()|indent(3) }}
      {% endfor %}
   {% endif %}
{% endif %}
