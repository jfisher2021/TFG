#!/usr/bin/env python3
"""
Analizador de experimentos v2 (planner + validator por ejecución)
Lee experimentos_pddl_v2.csv y reporta métricas por pareja de modelos y agregados.

Uso:
    python analisis_experimentos_v2.py [ruta_csv]
"""
from __future__ import annotations
import sys
from pathlib import Path
import pandas as pd

DEFAULT_CSV = Path("experimentos_pddl_v2.csv")

BOOL_MAP = {
    "S": True, "SI": True, "Y": True, "YES": True, "TRUE": True, "1": True,
    "N": False, "NO": False, "FALSE": False, "0": False,
    "NA": None, "": None, "NONE": None, "-": None
}

PAIRS = (
    ("Planner_Plan_Valido", "planner_plan_valido"),
    ("Planner_Formato_Valido", "planner_formato_valido"),
    ("Validator_Formato_Arreglado", "validator_formato_arreglado"),
    ("Validator_Mantiene_Plan", "validator_mantiene_plan"),
    ("Final_Plan_Valido", "final_plan_valido"),
)


def _find_col(df: pd.DataFrame, names: list[str]) -> str | None:
    cols = {c.lower(): c for c in df.columns}
    for n in names:
        if n.lower() in cols:
            return cols[n.lower()]
    return None


def _normalize_bool(s: pd.Series) -> pd.Series:
    # Mapea S/N/NA a True/False/NaN, preservando NaN
    mapped = s.astype(str).str.strip().str.upper().map(BOOL_MAP)
    return pd.Series(mapped, index=s.index, dtype="object")


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    # normalizar columnas clave de forma tolerante a mayúsculas/minúsculas
    planner_col = _find_col(df, ["Planner_Modelo"]) or "Planner_Modelo"
    validator_col = _find_col(df, ["Validator_Modelo"]) or "Validator_Modelo"

    # normalizar booleanos
    for raw_col, _alias in PAIRS:
        col = _find_col(df, [raw_col])
        if col and col in df.columns:
            df[col] = _normalize_bool(df[col])

    # renombrar a alias internos si existen
    rename_map = {}
    for raw_col, alias in PAIRS:
        col = _find_col(df, [raw_col])
        if col:
            rename_map[col] = alias
    if rename_map:
        df = df.rename(columns=rename_map)

    # asegurar que columnas de modelo existen
    if planner_col not in df.columns or validator_col not in df.columns:
        raise ValueError("Faltan columnas Planner_Modelo o Validator_Modelo en el CSV v2")
    df = df.rename(columns={planner_col: "Planner_Modelo", validator_col: "Validator_Modelo"})
    return df


def _ratio(series: pd.Series) -> float:
    # calcula % True sobre valores no nulos (ignora NaN)
    if series is None or len(series) == 0:
        return 0.0
    s = series.dropna()
    if len(s) == 0:
        return 0.0
    return float((s == True).mean() * 100.0)


def summarize_pairs(df: pd.DataFrame) -> pd.DataFrame:
    grp = df.groupby(["Planner_Modelo", "Validator_Modelo"], dropna=False)
    rows = []
    for (planner, validator), sub in grp:
        intentos = len(sub)
        pct_planner_val = _ratio(sub.get("planner_plan_valido"))
        pct_planner_fmt = _ratio(sub.get("planner_formato_valido"))
        pct_val_fixfmt = _ratio(sub.get("validator_formato_arreglado"))
        pct_val_keep = _ratio(sub.get("validator_mantiene_plan"))
        pct_final_val = _ratio(sub.get("final_plan_valido"))
        rows.append({
            "Planner_Modelo": planner,
            "Validator_Modelo": validator,
            "Intentos": intentos,
            "%PlannerPlanValido": round(pct_planner_val, 1),
            "%PlannerFormatoValido": round(pct_planner_fmt, 1),
            "%ValidadorCorrigeFormato": round(pct_val_fixfmt, 1),
            "%ValidadorMantienePlan": round(pct_val_keep, 1),
            "%FinalPlanValido": round(pct_final_val, 1),
        })
    out = pd.DataFrame(rows)
    if not out.empty:
        out = out.sort_values(["%FinalPlanValido", "%ValidadorMantienePlan", "Intentos"], ascending=[False, False, False])
    return out


def summarize_single(df: pd.DataFrame, col: str, label: str) -> pd.DataFrame:
    grp = df.groupby(col, dropna=False)
    rows = []
    for key, sub in grp:
        rows.append({
            label: key,
            "Intentos": len(sub),
            "%FinalPlanValido": round(_ratio(sub.get("final_plan_valido")), 1),
            "%CorrigeFormato": round(_ratio(sub.get("validator_formato_arreglado")), 1) if col == "Validator_Modelo" else round(_ratio(sub.get("planner_formato_valido")), 1),
        })
    out = pd.DataFrame(rows)
    if not out.empty:
        out = out.sort_values(["%FinalPlanValido", "Intentos"], ascending=[False, False])
    return out


def main(argv: list[str]):
    path = Path(argv[1]) if len(argv) > 1 else DEFAULT_CSV
    if not path.exists():
        print(f"No se encontró {path}. Pon el CSV v2 en la raíz del proyecto o pásalo por argumento.")
        return
    df = load_csv(path)

    print("\n=== Resumen por pareja Planner/Validator ===")
    pairs = summarize_pairs(df)
    if pairs.empty:
        print("Sin datos")
    else:
        print(pairs.to_string(index=False))

    print("\n=== Resumen por Planner ===")
    by_planner = summarize_single(df, "Planner_Modelo", "Planner")
    if by_planner.empty:
        print("Sin datos")
    else:
        print(by_planner.to_string(index=False))

    print("\n=== Resumen por Validator ===")
    by_validator = summarize_single(df, "Validator_Modelo", "Validator")
    if by_validator.empty:
        print("Sin datos")
    else:
        print(by_validator.to_string(index=False))


if __name__ == "__main__":
    main(sys.argv)
