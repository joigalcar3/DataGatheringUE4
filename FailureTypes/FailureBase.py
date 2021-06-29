import abc
import airsim


class FailureBase(abc.ABC):
    @staticmethod
    @abc.abstractmethod
    def activate_failure() -> None:
        """
        Method which injects the failure.
        :param client: the airsim client
        :param failure_mode: the chosen failure mode within the current failure type.
        It is a local identifier of the failure
        :return:
        """
        pass

    @staticmethod
    @abc.abstractmethod
    def mode_printer(client: airsim.MultirotorClient, failure_mode: int) -> str:
        """
        Method which prints information about the chosen failure mode.
        :param mode: the chosen mode of failure within the current failure type.
        :return:
        """
        pass
