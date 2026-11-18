#!/bin/sh
exec "${G2_PRODUCTION_MATCHER:?set G2_PRODUCTION_MATCHER}" --threads 4 --roi-reuse-m 10
