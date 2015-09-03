### Laelaps laelaps_control/ProductInfo message
#
# Product identifications.
#

# hardware versions
string  version_string    # Hardware version string.
int64   maj               # Hardware major version.
int64   min               # Hardware minor version.
int64   rev               # Hardware revision.

# product ids and description
string  product_name      # Product name.
uint32  product_id        # Product unique id.
string  desc              # Product description.

# environment
string  hostname          # Hostname of Laelaps.
