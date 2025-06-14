/*-----------------------------------------------------------/
/ set-version/index.js
/------------------------------------------------------------/
/ Copyright (c) 2025, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/-----------------------------------------------------------*/

const core = require('@actions/core');
const fs = require('fs');

try {
  const targetFile = core.getInput('target', {required: true});
  let text = fs.readFileSync(targetFile, 'utf-8');
  let version_str = core.getInput('version_str', {required: true});
  let version_str_size = core.getInput('version_str_size', {required: true});
  version_str = version_str.length > version_str_size ? version_str.slice(0, version_str_size) : version_str;
  let regExp = new RegExp(core.getInput('regexp_str', {required: true}));
  //let regExp = new RegExp('("CFG_ID_VERSION",\\s+)"\\d\.\\d\.\\d"');
  //let regExp = /("CFG_ID_VERSION",\s+)"\d\.\d\.\d"/;
  fs.writeFileSync(targetFile, text.replace(regExp, `$1"${version_str}"`));
  //fs.writeFileSync(targetFile, text.replace(/("CFG_ID_VERSION",\s+)"\d\.\d\.\d"/, `$1"${version_str}"`));
  //fs.writeFileSync(targetFile, text.replace(/("CFG_VERSION",\s+)"\d\.\d\.\d"/,  `$1"${version_str}"`));
} catch (error) {
    core.setFailed(error.message);
}