#!/usr/bin/env python3
"""
Analizador simple de resultados de planes PDDL guardados en experimentos_pddl.csv

Ahora soporta la columna adicional 'Fecha' y la columna 'Cumple_goal'.
Uso:
    python analisis_experimentos.py
"""
import pandas as pd
from pathlib import Path

CSV_PATH = Path("experimentos_pddl.csv")


BOOL_MAP = {
    "S": True, "SI": True, "Y": True, "YES": True, "TRUE": True, "1": True,
    "N": False, "NO": False, "FALSE": False, "0": False
}


def _find_col(df, name_candidates):
    cols = {c.lower(): c for c in df.columns}
    for cand in name_candidates:
        if cand.lower() in cols:
            return cols[cand.lower()]
    return None


def _normalize_bool_series(s: pd.Series) -> pd.Series:
    return s.astype(str).str.strip().str.upper().map(BOOL_MAP).fillna(False)


def main():
    if not CSV_PATH.exists():
        print(f"No se encontró {CSV_PATH}. Coloca el CSV en el mismo directorio.")
        return

    df = pd.read_csv(CSV_PATH)

    # detectar columnas relevantes de forma tolerante (mayúsculas/minúsculas)
    modelo_col = _find_col(df, ["Modelo", "modelo"])
    formato_col = _find_col(df, ["Formato_valido", "Formato_valido", "formato_valido", "Formato_valídO"])
    plan_col = _find_col(df, ["Plan_Valido", "Plan_valido", "plan_valido"])
    cumple_col = _find_col(df, ["Cumple_goal", "cumple_goal", "Cumple_Goal"])
    fecha_col = _find_col(df, ["Fecha", "fecha"])

    if modelo_col is None:
        print("Falta la columna 'Modelo' en el CSV.")
        return

    # Normalizar booleanos si existen
    if formato_col:
        df[formato_col] = _normalize_bool_series(df[formato_col])
    if plan_col:
        df[plan_col] = _normalize_bool_series(df[plan_col])
    if cumple_col:
        df[cumple_col] = _normalize_bool_series(df[cumple_col])

    # Parsear fecha si está presente
    if fecha_col:
        try:
            df[fecha_col] = pd.to_datetime(df[fecha_col], errors="coerce")
        except Exception:
            pass

    # Métricas por modelo
    agg_kwargs = {
        "intentos": ("Modelo", "size"),
    }
    if plan_col:
        agg_kwargs["planes_validos"] = (plan_col, "sum")
    else:
        df["__plan_dummy__"] = False
        agg_kwargs["planes_validos"] = ("__plan_dummy__", "sum")
    if formato_col:
        agg_kwargs["formatos_validos"] = (formato_col, "sum")
    else:
        df["__form_dummy__"] = False
        agg_kwargs["formatos_validos"] = ("__form_dummy__", "sum")
    if cumple_col:
        agg_kwargs["cumple_goal"] = (cumple_col, "sum")
    
    resumen = (
        df
        .groupby(modelo_col)
        .agg(**agg_kwargs)
        .assign(
            pct_formato_valido=lambda x: (x["formatos_validos"] / x["intentos"] * 100).round(1),
            pct_plan_valido=lambda x: (x["planes_validos"] / x["intentos"] * 100).round(1),
        )
        .sort_values(by=["pct_plan_valido", "pct_formato_valido", "intentos"], ascending=[False, False, False])
    )

    print("\n=== Resumen por modelo ===")
    print(resumen.to_string())

    # ==========================
    # Sección por Goal individual
    # ==========================
    goal_col = _find_col(df, ["Goal", "goal"])
    if goal_col:
        goals = (
            df[goal_col]
            .astype(str)
            .str.strip()
            .replace({"": None})
            .dropna()
            .unique()
            .tolist()
        )

        print("\n=== Resumen por modelo dividido por Goal ===")
        for g in goals:
            sub = df[df[goal_col].astype(str).str.strip() == g]
            if sub.empty:
                continue
            # construir agg_kwargs para el subconjunto (mismas reglas de arriba)
            agg_kwargs_g = {"intentos": (modelo_col, "size")}
            if plan_col:
                agg_kwargs_g["planes_validos"] = (plan_col, "sum")
            else:
                sub["__plan_dummy__"] = False
                agg_kwargs_g["planes_validos"] = ("__plan_dummy__", "sum")
            if formato_col:
                agg_kwargs_g["formatos_validos"] = (formato_col, "sum")
            else:
                sub["__form_dummy__"] = False
                agg_kwargs_g["formatos_validos"] = ("__form_dummy__", "sum")
            if cumple_col:
                agg_kwargs_g["cumple_goal"] = (cumple_col, "sum")

            resumen_g = (
                sub
                .groupby(modelo_col)
                .agg(**agg_kwargs_g)
                .assign(
                    pct_formato_valido=lambda x: (x["formatos_validos"] / x["intentos"] * 100).round(1),
                    pct_plan_valido=lambda x: (x["planes_validos"] / x["intentos"] * 100).round(1),
                )
                .sort_values(by=["pct_plan_valido", "pct_formato_valido", "intentos"], ascending=[False, False, False])
            )
            print(f"\n--- Goal: {g} ---")
            print(resumen_g.to_string())


if __name__ == "__main__":
    main()