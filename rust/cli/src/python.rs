use pep440_rs::Version;
use pep508_rs::{PackageName, Requirement, VersionOrUrl};

use crate::sh;
use std::{
    collections::{BTreeMap, BTreeSet}, error::Error,
};

const LIST_INSTALLED_PACKAGES: &str = r#"
import importlib.metadata as m

packages = {
    (d.name, d.version)
    for d in m.distributions()
    if d.name
}

for name, version in sorted(packages):
    print(f"{name}=={version}")
"#;

/// Normalized names of python packages visible to python3.
pub fn installed_packages(host: Option<&str>) -> Result<BTreeMap<PackageName, Version>, Box<dyn Error>> {
    let command = format!(
        "python3 - <<'PENNAIR_PYTHON'\n{LIST_INSTALLED_PACKAGES}\nPENNAIR_PYTHON"
    );
    let out = sh::output(host, &command)?;

    let mut packages = BTreeMap::new();

    for line in out.lines() {
        let (name, version) = line.split_once("==").ok_or("expected format package==version")?;
        let name: PackageName = name.parse()?;
        let version: Version = version.parse()?;
        packages.insert(name, version);
    }
    Ok(packages)
}

/// Uses pep508_rs to parse all python requirement strings
pub fn parse_requirements(requirements: &BTreeSet<String>) -> Result<BTreeSet<Requirement>, Box<dyn Error>> {
    let mut parsed = BTreeSet::new();
    for requirement in requirements {
        let requirement = requirement
            .parse::<Requirement>()
            .map_err(|e| format!("Invalid Python requirement '{requirement}': {e}"))?;
        parsed.insert(requirement);
    }
    Ok(parsed)
}

/// Checks a set of parsed requirements against detected python packages. Returns a set of
/// unsatisfied requirements if successful
pub fn find_unsatisfied_requirements(required: &BTreeSet<Requirement>, installed: &BTreeMap<PackageName, Version>) -> Result<BTreeSet<Requirement>, Box<dyn Error>>{

    let mut unsatisfied = BTreeSet::new();
    for requirement in required {

        // checks if installed has a package that satisfies each requirement
        let satisfied = match installed.get(&requirement.name) {
            None => false,
            Some(version) => match &requirement.version_or_url {
                None => true,
                Some(VersionOrUrl::VersionSpecifier(specifiers)) => {
                    specifiers.contains(version) // only pass if the specifiers allow the installed
                                                 // version
                }
                Some(VersionOrUrl::Url(_)) => { // don't allow URLs
                    return Err("URL requirements are unsupported".into());
                }
            },
        };
        if !satisfied {
            unsatisfied.insert(requirement.clone());
        }
    }
    Ok(unsatisfied)
}
