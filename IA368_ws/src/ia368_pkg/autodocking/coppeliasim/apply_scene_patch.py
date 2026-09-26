"""Instala python_controler.py (desta pasta) no script /myRobot/python_controler da cena.

Uso (com a cena "Evaluation scene3.2_students.ttt" aberta no CoppeliaSim e a
simulação PARADA):

    python3 apply_scene_patch.py          # instala o script na cena aberta
    python3 apply_scene_patch.py --save   # instala e salva o .ttt

Antes de trocar, o código atual da cena é salvo em
python_controler.scene_backup.py, nesta pasta.
"""
import argparse
import pathlib
import sys

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

HERE = pathlib.Path(__file__).resolve().parent
SCRIPT_PATH = '/myRobot/python_controler'


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--save', action='store_true', help='salva a cena depois de instalar')
    args = parser.parse_args()

    sim = RemoteAPIClient().require('sim')

    scene = sim.getStringParam(sim.stringparam_scene_path_and_name)
    print(f'Cena aberta: {scene}')
    if 'Evaluation scene3.2_students' not in scene:
        sys.exit('Abra a cena "Evaluation scene3.2_students.ttt" antes de rodar este script.')
    if sim.getSimulationState() != sim.simulation_stopped:
        sys.exit('Pare a simulação antes de rodar este script.')

    script = sim.getObject(SCRIPT_PATH)
    old_code = sim.getObjectStringParam(script, sim.scriptstringparam_text)
    if isinstance(old_code, bytes):
        old_code = old_code.decode()
    new_code = (HERE / 'python_controler.py').read_text()

    # Scripts Python da cena podem começar com a linha "#python", que indica a
    # linguagem ao CoppeliaSim; ela precisa ser preservada.
    first_line = old_code.splitlines()[0] if old_code else ''
    if first_line.startswith('#python') and not new_code.startswith('#python'):
        new_code = first_line + '\n' + new_code

    if old_code == new_code:
        print('O script da cena já está nesta versão. Nada a fazer.')
    else:
        backup = HERE / 'python_controler.scene_backup.py'
        backup.write_text(old_code)
        print(f'Código anterior salvo em {backup}')
        sim.setObjectStringParam(script, sim.scriptstringparam_text, new_code)
        print(f'Script {SCRIPT_PATH} atualizado.')

    if args.save:
        sim.saveScene(scene)
        print(f'Cena salva em {scene}')
    else:
        print('Cena NÃO salva (use --save, ou salve pelo CoppeliaSim).')


if __name__ == '__main__':
    main()
